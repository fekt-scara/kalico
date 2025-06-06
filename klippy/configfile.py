# Code for reading and writing the Klipper config file
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import sys, os, glob, re, time, logging, configparser, io
import pathlib
from .extras.danger_options import get_danger_options
from . import mathutil


error = configparser.Error


class sentinel:
    pass


PYTHON_SCRIPT_PREFIX = "!"
_INCLUDERE = re.compile(r"!!include (?P<file>.*)")


def _fix_include_path(source_file: str, match: re.Match) -> pathlib.Path:
    new_path = pathlib.Path(source_file).parent.absolute() / match.group("file")
    if not new_path.is_file():
        raise error(f"Attempted to include non-existent file {new_path}")
    return f"!!include {new_path}"


class SectionInterpolation(configparser.Interpolation):
    """
    variable interpolation replacing ${[section.]option}
    """

    _KEYCRE = re.compile(
        r"\$\{(?:(?P<section>[^.:${}]+)[.:])?(?P<option>[^${}]+)\}"
    )

    def __init__(self, access_tracking):
        self.access_tracking = access_tracking

    def before_get(self, parser, section, option, value, defaults):
        if not isinstance(value, str):
            return value
        depth = configparser.MAX_INTERPOLATION_DEPTH
        while depth:
            depth -= 1

            match = self._KEYCRE.search(value)
            if not match:
                break

            sect = match.group("section") or section
            opt = match.group("option")

            const = parser.get(sect, opt)
            self.access_tracking.setdefault((sect, opt), const)

            value = value[: match.start()] + const + value[match.end() :]

        return value


class ConfigWrapper:
    error = configparser.Error

    def __init__(self, printer, fileconfig, access_tracking, section):
        self.printer = printer
        self.fileconfig = fileconfig
        self.access_tracking = access_tracking
        self.section = section

    def get_printer(self):
        return self.printer

    def get_name(self):
        return self.section

    def _get_wrapper(
        self,
        parser,
        option,
        default,
        minval=None,
        maxval=None,
        above=None,
        below=None,
        note_valid=True,
    ):
        if not self.fileconfig.has_option(self.section, option):
            if default is not sentinel:
                if note_valid and default is not None:
                    acc_id = (self.section.lower(), option.lower())
                    self.access_tracking[acc_id] = default
                return default
            raise error(
                "Option '%s' in section '%s' must be specified"
                % (option, self.section)
            )
        if parser is float:
            parser = mathutil.safe_float
        try:
            v = parser(self.section, option)
        except self.error as e:
            raise
        except:
            raise error(
                "Unable to parse option '%s' in section '%s'"
                % (option, self.section)
            )
        if note_valid:
            self.access_tracking[(self.section.lower(), option.lower())] = v
        if minval is not None and v < minval:
            raise error(
                "Option '%s' in section '%s' must have minimum of %s"
                % (option, self.section, minval)
            )
        if maxval is not None and v > maxval:
            raise error(
                "Option '%s' in section '%s' must have maximum of %s"
                % (option, self.section, maxval)
            )
        if above is not None and v <= above:
            raise error(
                "Option '%s' in section '%s' must be above %s"
                % (option, self.section, above)
            )
        if below is not None and v >= below:
            raise self.error(
                "Option '%s' in section '%s' must be below %s"
                % (option, self.section, below)
            )
        return v

    def get(self, option, default=sentinel, note_valid=True):
        return self._get_wrapper(
            self.fileconfig.get, option, default, note_valid=note_valid
        )

    def getscript(self, option, default=sentinel, note_valid=True):
        value: str = self.get(option, default, note_valid).strip()

        match = _INCLUDERE.search(value)
        if match:
            file_path = pathlib.Path(match.group("file"))
            if file_path.suffix.lower() == ".py":
                return ("python", file_path.read_text())
            else:
                return ("gcode", file_path.read_text())

        elif value.startswith(PYTHON_SCRIPT_PREFIX):
            return (
                "python",
                "\n".join(
                    line.removeprefix(PYTHON_SCRIPT_PREFIX)
                    for line in value.splitlines()
                ),
            )

        return ("gcode", value)

    def getint(
        self,
        option,
        default=sentinel,
        minval=None,
        maxval=None,
        note_valid=True,
    ):
        return self._get_wrapper(
            self.fileconfig.getint,
            option,
            default,
            minval,
            maxval,
            note_valid=note_valid,
        )

    def getfloat(
        self,
        option,
        default=sentinel,
        minval=None,
        maxval=None,
        above=None,
        below=None,
        note_valid=True,
    ):
        return self._get_wrapper(
            self.fileconfig.getfloat,
            option,
            default,
            minval,
            maxval,
            above,
            below,
            note_valid=note_valid,
        )

    def getboolean(self, option, default=sentinel, note_valid=True):
        return self._get_wrapper(
            self.fileconfig.getboolean, option, default, note_valid=note_valid
        )

    def getchoice(self, option, choices, default=sentinel, note_valid=True):
        if isinstance(choices, list):
            choices = {i: i for i in choices}
        if choices and isinstance(list(choices.keys())[0], int):
            c = self.getint(option, default, note_valid=note_valid)
        else:
            c = self.get(option, default, note_valid=note_valid)
        if c not in choices:
            raise error(
                "Choice '%s' for option '%s' in section '%s'"
                " is not a valid choice" % (c, option, self.section)
            )
        return choices[c]

    def getlists(
        self,
        option,
        default=sentinel,
        seps=(",",),
        count=None,
        parser=str,
        note_valid=True,
    ):
        def lparser(value, pos):
            if len(value.strip()) == 0:
                # Return an empty list instead of [''] for empty string
                parts = []
            else:
                parts = [p.strip() for p in value.split(seps[pos])]
            if pos:
                # Nested list
                return tuple([lparser(p, pos - 1) for p in parts if p])
            res = [parser(p) for p in parts]
            if count is not None and len(res) != count:
                raise error(
                    "Option '%s' in section '%s' must have %d elements"
                    % (option, self.section, count)
                )
            return tuple(res)

        def fcparser(section, option):
            return lparser(self.fileconfig.get(section, option), len(seps) - 1)

        return self._get_wrapper(
            fcparser, option, default, note_valid=note_valid
        )

    def getlist(
        self, option, default=sentinel, sep=",", count=None, note_valid=True
    ):
        return self.getlists(
            option,
            default,
            seps=(sep,),
            count=count,
            parser=str,
            note_valid=note_valid,
        )

    def getintlist(
        self, option, default=sentinel, sep=",", count=None, note_valid=True
    ):
        return self.getlists(
            option,
            default,
            seps=(sep,),
            count=count,
            parser=int,
            note_valid=note_valid,
        )

    def getfloatlist(
        self, option, default=sentinel, sep=",", count=None, note_valid=True
    ):
        return self.getlists(
            option,
            default,
            seps=(sep,),
            count=count,
            parser=mathutil.safe_float,
            note_valid=note_valid,
        )

    def getsection(self, section):
        return ConfigWrapper(
            self.printer, self.fileconfig, self.access_tracking, section
        )

    def has_section(self, section):
        return self.fileconfig.has_section(section)

    def get_prefix_sections(self, prefix):
        return [
            self.getsection(s)
            for s in self.fileconfig.sections()
            if s.startswith(prefix)
        ]

    def get_prefix_options(self, prefix):
        return [
            o
            for o in self.fileconfig.options(self.section)
            if o.startswith(prefix)
        ]

    def deprecate(self, option, value=None):
        if not self.fileconfig.has_option(self.section, option):
            return
        if value is None:
            msg = "Option '%s' in section '%s' is deprecated." % (
                option,
                self.section,
            )
        else:
            msg = "Value '%s' in option '%s' in section '%s' is deprecated." % (
                value,
                option,
                self.section,
            )
        pconfig = self.printer.lookup_object("configfile")
        pconfig.deprecate(self.section, option, value, msg)


AUTOSAVE_HEADER = """
#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
"""


class PrinterConfig:
    def __init__(self, printer):
        self.printer = printer
        self.autosave = None
        self.deprecated = {}
        self.runtime_warnings = []
        self.deprecate_warnings = []
        self.status_raw_config = {}
        self.status_save_pending = {}
        self.status_settings = {}
        self.status_warnings = []
        self.unused_sections = []
        self.unused_options = []
        self.save_config_pending = False
        gcode = self.printer.lookup_object("gcode")
        if "SAVE_CONFIG" not in gcode.ready_gcode_handlers:
            gcode.register_command(
                "SAVE_CONFIG",
                self.cmd_SAVE_CONFIG,
                desc=self.cmd_SAVE_CONFIG_help,
            )

    def get_printer(self):
        return self.printer

    def _read_config_file(self, filename):
        try:
            f = open(filename, "r")
            data = f.read()
            f.close()
        except:
            msg = "Unable to open config file %s" % (filename,)
            logging.exception(msg)
            raise error(msg)
        return data.replace("\r\n", "\n")

    def _find_autosave_data(self, data):
        regular_data = data
        autosave_data = ""
        pos = data.find(AUTOSAVE_HEADER)
        if pos >= 0:
            regular_data = data[:pos]
            autosave_data = data[pos + len(AUTOSAVE_HEADER) :].strip()
        # Check for errors and strip line prefixes
        if "\n#*# " in regular_data:
            logging.warning(
                "Can't read autosave from config file"
                " - autosave state corrupted"
            )
            return data, ""
        out = [""]
        for line in autosave_data.split("\n"):
            if (
                not line.startswith("#*#")
                or (len(line) >= 4 and not line.startswith("#*# "))
            ) and autosave_data:
                logging.warning(
                    "Can't read autosave from config file"
                    " - modifications after header"
                )
                return data, ""
            out.append(line[4:])
        out.append("")
        return regular_data, "\n".join(out)

    comment_r = re.compile("[#;].*$")
    value_r = re.compile("[^A-Za-z0-9_].*$")

    def _strip_duplicates(self, data, config):
        # Comment out fields in 'data' that are defined in 'config'
        lines = data.split("\n")
        section = None
        is_dup_field = False
        for lineno, line in enumerate(lines):
            pruned_line = self.comment_r.sub("", line).rstrip()
            if not pruned_line:
                continue
            if pruned_line[0].isspace():
                if is_dup_field:
                    lines[lineno] = "#" + lines[lineno]
                continue
            is_dup_field = False
            if pruned_line[0] == "[":
                section = pruned_line[1:-1].strip()
                continue
            field = self.value_r.sub("", pruned_line)
            if config.fileconfig.has_option(section, field):
                is_dup_field = True
                lines[lineno] = "#" + lines[lineno]
        return "\n".join(lines)

    def _parse_config_buffer(self, buffer, filename, fileconfig):
        if not buffer:
            return
        data = "\n".join(buffer)
        del buffer[:]
        sbuffer = io.StringIO(data)
        if sys.version_info.major >= 3:
            fileconfig.read_file(sbuffer, filename)
        else:
            fileconfig.readfp(sbuffer, filename)

    def _resolve_include(
        self, source_filename, include_spec, fileconfig, visited
    ):
        dirname = os.path.dirname(source_filename)
        include_spec = include_spec.strip()
        include_glob = os.path.join(dirname, include_spec)
        if sys.version_info >= (3, 5):
            include_filenames = glob.glob(include_glob, recursive=True)
        else:
            include_filenames = glob.glob(include_glob)
        if not include_filenames and not glob.has_magic(include_glob):
            # Empty set is OK if wildcard but not for direct file reference
            raise error("Include file '%s' does not exist" % (include_glob,))
        include_filenames.sort()
        for include_filename in include_filenames:
            include_data = self._read_config_file(include_filename)
            self._parse_config(
                include_data, include_filename, fileconfig, visited
            )
        return include_filenames

    def _parse_config(self, data, filename, fileconfig, visited):
        path = os.path.abspath(filename)
        if path in visited:
            raise error("Recursive include of config file '%s'" % (filename))
        visited.add(path)
        lines = data.split("\n")
        # Buffer lines between includes and parse as a unit so that overrides
        # in includes apply linearly as they do within a single file
        buffer = []
        for line in lines:
            # Strip trailing comment
            pos = line.find("#")
            if pos >= 0:
                line = line[:pos]
            # Process include or buffer line
            mo = configparser.RawConfigParser.SECTCRE.match(line)
            header = mo and mo.group("header")
            if header and header.startswith("include "):
                self._parse_config_buffer(buffer, filename, fileconfig)
                include_spec = header[8:].strip()
                self._resolve_include(
                    filename, include_spec, fileconfig, visited
                )
            else:
                line = _INCLUDERE.sub(
                    lambda match: _fix_include_path(filename, match),
                    line,
                )
                buffer.append(line)
        self._parse_config_buffer(buffer, filename, fileconfig)
        visited.remove(path)

    def _build_config_wrapper(self, data, filename):
        access_tracking = {}
        fileconfig = configparser.RawConfigParser(
            strict=False,
            inline_comment_prefixes=(";", "#"),
            interpolation=SectionInterpolation(access_tracking),
        )

        self._parse_config(data, filename, fileconfig, set())
        return ConfigWrapper(
            self.printer, fileconfig, access_tracking, "printer"
        )

    def _build_config_string(self, config):
        sfile = io.StringIO()
        config.fileconfig.write(sfile)
        return sfile.getvalue().strip()

    def read_config(self, filename):
        return self._build_config_wrapper(
            self._read_config_file(filename), filename
        )

    def read_main_config(self):
        filename = self.printer.get_start_args()["config_file"]
        data = self._read_config_file(filename)
        regular_data, autosave_data = self._find_autosave_data(data)
        regular_config = self._build_config_wrapper(regular_data, filename)
        autosave_data = self._strip_duplicates(autosave_data, regular_config)
        self.autosave = self._build_config_wrapper(autosave_data, filename)
        cfg = self._build_config_wrapper(regular_data + autosave_data, filename)
        return cfg

    def check_unused_options(self, config, error_on_unused):
        fileconfig = config.fileconfig
        objects = dict(self.printer.lookup_objects())
        # Determine all the fields that have been accessed
        access_tracking = dict(config.access_tracking)
        for section in self.autosave.fileconfig.sections():
            for option in self.autosave.fileconfig.options(section):
                access_tracking[(section.lower(), option.lower())] = 1
        # Validate that there are no undefined parameters in the config file
        valid_sections = {s for s, o in access_tracking}
        for section_name in fileconfig.sections():
            section = section_name.lower()
            if section not in valid_sections and section not in objects:
                if error_on_unused:
                    raise error(
                        "Section '%s' is not a valid config section"
                        % (section,)
                    )
                else:
                    self.unused_sections.append(section)
            for option in fileconfig.options(section_name):
                option = option.lower()
                if (section, option) not in access_tracking:
                    if error_on_unused and section != "constants":
                        raise error(
                            "Option '%s' is not valid in section '%s'"
                            % (option, section)
                        )
                    else:
                        self.unused_options.append((section, option))
        # Setup get_status()
        self._build_status(config)

    def log_config(self, config):
        lines = [
            "===== Config file =====",
            self._build_config_string(config),
            "=======================",
        ]
        self.printer.set_rollover_info("config", "\n".join(lines))

    # Status reporting
    def runtime_warning(self, msg):
        logging.warning(msg)
        res = {"type": "runtime_warning", "message": msg}
        self.runtime_warnings.append(res)
        self.status_warnings = self.runtime_warnings + self.deprecate_warnings

    def deprecate(self, section, option, value=None, msg=None):
        self.deprecated[(section, option, value)] = msg

    def warn(self, type, msg, section=None, option=None, value=None):
        res = {
            "type": type,
            "message": msg,
        }
        if section is not None:
            res["section"] = section
        if option is not None:
            res["option"] = option
        if value is not None:
            res["value"] = value
        self.status_warnings.append(res)

    def _build_status(self, config):
        self.status_raw_config.clear()
        for section in config.get_prefix_sections(""):
            self.status_raw_config[section.get_name()] = section_status = {}
            for option in section.get_prefix_options(""):
                section_status[option] = section.get(option, note_valid=False)
        self.status_settings = {}
        for (section, option), value in config.access_tracking.items():
            self.status_settings.setdefault(section, {})[option] = value
        for (section, option, value), msg in self.deprecated.items():
            _type = "deprecated_value"
            self.warn(_type, msg, section, option, value)

        for section, option in self.unused_options:
            _type = "unused_option"
            if section == "constants":
                msg = f"Constant '{option}' is unused"
            else:
                msg = f"Option '{option}' in section '{section}' is invalid"
            self.warn(_type, msg, section, option)
        for section in self.unused_sections:
            _type = "unused_section"
            msg = f"Section '{section}' is invalid"
            self.warn(_type, msg, section)

    def get_status(self, eventtime):
        return {
            "config": self.status_raw_config,
            "settings": self.status_settings,
            "warnings": self.status_warnings,
            "save_config_pending": self.save_config_pending,
            "save_config_pending_items": self.status_save_pending,
        }

    # Autosave functions
    def set(self, section, option, value):
        if not self.autosave.fileconfig.has_section(section):
            self.autosave.fileconfig.add_section(section)
        svalue = str(value)
        self.autosave.fileconfig.set(section, option, svalue)
        pending = dict(self.status_save_pending)
        if section not in pending or pending[section] is None:
            pending[section] = {}
        else:
            pending[section] = dict(pending[section])
        pending[section][option] = svalue
        self.status_save_pending = pending
        self.save_config_pending = True
        logging.info("save_config: set [%s] %s = %s", section, option, svalue)

    def remove_section(self, section):
        if self.autosave.fileconfig.has_section(section):
            self.autosave.fileconfig.remove_section(section)
            pending = dict(self.status_save_pending)
            pending[section] = None
            self.status_save_pending = pending
            self.save_config_pending = True
        elif (
            section in self.status_save_pending
            and self.status_save_pending[section] is not None
        ):
            pending = dict(self.status_save_pending)
            del pending[section]
            self.status_save_pending = pending
            self.save_config_pending = True

    def _disallow_include_conflicts(self, regular_data, cfgname, gcode):
        config = self._build_config_wrapper(regular_data, cfgname)
        for section in self.autosave.fileconfig.sections():
            for option in self.autosave.fileconfig.options(section):
                if config.fileconfig.has_option(section, option):
                    # They conflict only if they are not the same value
                    included_value = config.fileconfig.get(section, option)
                    autosave_value = self.autosave.fileconfig.get(
                        section, option
                    )
                    if included_value != autosave_value:
                        msg = (
                            "SAVE_CONFIG section '%s' option '%s' value '%s' conflicts "
                            "with included value '%s' "
                            % (section, option, autosave_value, included_value)
                        )
                        raise gcode.error(msg)

    cmd_SAVE_CONFIG_help = "Overwrite config file and restart"

    def _write_backup(self, cfgpath, cfgdata, gcode):
        printercfg = self.printer.get_start_args()["config_file"]
        configdir = os.path.dirname(printercfg)
        # Define a directory for configuration backups so that include blocks
        # using a wildcard to reference all files in a directory don't throw
        # errors
        backupdir = os.path.join(configdir, "config_backups")
        # Create the backup directory if it doesn't already exist
        if not os.path.exists(backupdir):
            os.mkdir(backupdir)

        # Generate the name of the backup file by stripping the leading path in
        # `cfgpath` and appending to it. Then add it to the config_backups dir
        datestr = time.strftime("-%Y%m%d_%H%M%S")
        cfgname = os.path.basename(cfgpath)
        backup_path = backupdir + "/" + cfgname + datestr
        if cfgpath.endswith(".cfg"):
            backup_path = backupdir + "/" + cfgname[:-4] + datestr + ".cfg"
        logging.info(
            "SAVE_CONFIG to '%s' (backup in '%s')", cfgpath, backup_path
        )
        try:
            # Read the current config into the backup before making changes to
            # the original file
            currentconfig = open(cfgpath, "r")
            backupconfig = open(backup_path, "w")
            backupconfig.write(currentconfig.read())
            backupconfig.close()
            currentconfig.close()
            # With the backup created, write the new data to the original file
            currentconfig = open(cfgpath, "w")
            currentconfig.write(cfgdata)
            currentconfig.close()
        except:
            msg = "Unable to write config file during SAVE_CONFIG"
            logging.exception(msg)
            raise gcode.error(msg)

    def _save_includes(self, cfgpath, data, visitedpaths, gcode):
        # Prevent an infinite loop in the event of configs circularly
        # referencing each other
        if cfgpath in visitedpaths:
            return

        visitedpaths.add(cfgpath)
        dirname = os.path.dirname(cfgpath)
        # Read the data as individual lines so we can find include blocks
        lines = data.split("\n")
        for line in lines:
            # Strip trailing comment
            pos = line.find("#")
            if pos >= 0:
                line = line[:pos]

            mo = configparser.RawConfigParser.SECTCRE.match(line)
            header = mo and mo.group("header")
            if header and header.startswith("include "):
                include_spec = header[8:].strip()
                include_glob = os.path.join(dirname, include_spec)
                # retrieve all filenames associated with the absolute path of
                # the include header
                include_filenames = glob.glob(include_glob)
                if not include_filenames and not glob.has_magic(include_glob):
                    # Empty set is OK if wildcard but not for direct file
                    # reference
                    raise error(
                        "Include file '%s' does not exist" % (include_glob,)
                    )
                include_filenames.sort()
                # Read the include files and check them against autosave data.
                # If autosave data overwites anything we'll update the file
                # and create a backup.
                for include_filename in include_filenames:
                    # Recursively check for includes. No need to check for looping
                    # includes as klipper checks this at startup.
                    include_predata = self._read_config_file(include_filename)
                    self._save_includes(
                        include_filename, include_predata, visitedpaths, gcode
                    )

                    include_postdata = self._strip_duplicates(
                        include_predata, self.autosave
                    )
                    # Only write and backup data that's been changed
                    if include_predata != include_postdata:
                        self._write_backup(
                            include_filename, include_postdata, gcode
                        )

    def cmd_SAVE_CONFIG(self, gcmd):
        if not self.autosave.fileconfig.sections():
            return
        gcode = self.printer.lookup_object("gcode")
        # Create string containing autosave data
        autosave_data = self._build_config_string(self.autosave)
        lines = [("#*# " + l).strip() for l in autosave_data.split("\n")]
        lines.insert(0, "\n" + AUTOSAVE_HEADER.rstrip())
        lines.append("")
        autosave_data = "\n".join(lines)
        # Read in and validate current config file
        cfgname = self.printer.get_start_args()["config_file"]
        try:
            data = self._read_config_file(cfgname)
            regular_data, old_autosave_data = self._find_autosave_data(data)
            config = self._build_config_wrapper(regular_data, cfgname)
        except error as e:
            msg = "Unable to parse existing config on SAVE_CONFIG"
            logging.exception(msg)
            raise gcode.error(msg)
        regular_data = self._strip_duplicates(regular_data, self.autosave)

        if get_danger_options().autosave_includes:
            self._save_includes(cfgname, data, set(), gcode)

        # NOW we're safe to check for conflicts
        self._disallow_include_conflicts(regular_data, cfgname, gcode)
        data = regular_data.rstrip() + autosave_data
        self._write_backup(cfgname, data, gcode)

        # If requested restart or no restart just flag config saved
        require_restart = gcmd.get_int("RESTART", 1, minval=0, maxval=1)
        if require_restart:
            # Request a restart
            gcode.request_restart("restart")
        else:
            # flag config updated to false since config saved with no restart
            self.save_config_pending = False
            gcode.respond_info("Config update without restart successful")

# Load the ESP-IDF environment for this project.
# Usage (from nus_flight_app):  source env.sh
#
# Must be sourced, not executed: running it as ./env.sh would set things up
# in a throwaway shell and leave your terminal unchanged.

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
    echo "env.sh must be sourced:  source env.sh" >&2
    exit 1
fi

IDF_ACTIVATE="$HOME/.espressif/tools/activate_idf_v6.0.2.sh"

if [ ! -f "$IDF_ACTIVATE" ]; then
    echo "ESP-IDF activation script not found: $IDF_ACTIVATE" >&2
    return 1
fi

source "$IDF_ACTIVATE" || return 1

# The activation script defines idf.py as a shell function, which child
# scripts (idf_logged.sh, prepare_experiment.py) cannot see. Put the real
# executable on PATH too; its `#!/usr/bin/env python` resolves to the venv.
if [ -n "$IDF_PATH" ] && [ -x "$IDF_PATH/tools/idf.py" ]; then
    case ":$PATH:" in
        *":$IDF_PATH/tools:"*) ;;
        *) export PATH="$IDF_PATH/tools:$PATH" ;;
    esac
fi

if type -P idf.py >/dev/null 2>&1; then
    echo "ESP-IDF ready: $(idf.py --version 2>/dev/null)"
else
    echo "ESP-IDF activation ran, but idf.py is still not on PATH" >&2
    return 1
fi

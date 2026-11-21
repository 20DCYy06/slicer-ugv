# Reuse the account's dialout membership in terminals opened before it was added.
run_serial_tool() {
    local argument groups
    for argument in "$@"; do
        case "$argument" in
            --help|-h|--self-test) exec "$@" ;;
        esac
    done

    groups=$(id -nG)
    if [[ " $groups " != *" dialout "* ]] && command -v sg >/dev/null; then
        groups=$(id -nG "$(id -un)")
        if [[ " $groups " == *" dialout "* ]]; then
            exec /usr/bin/python3 -s -c 'import os, shlex, sys
os.execlp("sg", "sg", "dialout", "-c", "exec " + shlex.join(sys.argv[1:]))' "$@"
        fi
    fi
    exec "$@"
}

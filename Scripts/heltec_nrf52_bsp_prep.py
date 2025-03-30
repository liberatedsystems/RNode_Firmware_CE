#!/usr/bin/env python

# This is a helper script to perform some additional preparation steps before
# building.  This helper script may be deleted once some upstream bugs go away.
# Tested with Python 3.13.2 on Linux.
#   See https://github.com/liberatedsystems/RNode_Firmware_CE/blob/master/Documentation/BUILDING.md

# There's a minor issue when building images for "Heltec T114" boards.  The fix
# has been merged but there's no release after "1.7.0" containing this yet.  As
# a result, we need to correct some files between the "make prep-nrf" and "make
# firmware-heltec_t114_gps" steps.
#   See https://github.com/HelTecAutomation/Heltec_nRF52/issues/4
#   See https://github.com/HelTecAutomation/Heltec_nRF52/pull/3

# An equivalent shell command for doing this would be the following (GNU sed, not BSD sed):
#   find ~/.arduino15/packages/Heltec_nRF52 -name 'platform.txt' | xargs \
#     sed -i 's/recipe\.objcopy\.uf2\.pattern="{tools\.uf2conv\.cmd}"/recipe\.objcopy\.uf2\.pattern={tools\.uf2conv\.cmd}/'


from fnmatch import fnmatch
from os import environ, path, walk
from re import sub
from shutil import move
from tempfile import mkstemp


def find_all_files_to_fix() -> list:
    found = []
    for root, dirs, files in walk(
        f'{environ["HOME"]}/.arduino15/packages/Heltec_nRF52'
    ):
        for file in files:
            if fnmatch(file, 'platform.txt'):
                found.append(path.join(root, file))
    return found


def sed_fix_files_in_place(files: list = []) -> None:
    for file in files:
        fd, temp = mkstemp()
        with open(file, 'r') as source, open(temp, 'w') as dest:
            for line in source:
                out = sub(
                    r'recipe.objcopy.uf2.pattern="{tools.uf2conv.cmd}"',
                    r'recipe.objcopy.uf2.pattern={tools.uf2conv.cmd}',
                    line,
                )
                dest.write(out)
        move(temp, file)


if '__main__' == __name__:
    sed_fix_files_in_place(find_all_files_to_fix())

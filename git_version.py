#import sys
#from io import StringIO
import subprocess

result = subprocess.run("git describe --abbrev=6 --dirty --always --tags", 
                        shell=True, 
                        capture_output = True,
                        text=True)

git_versio = result.stdout
print("GIT Version: ", git_versio)

fh = open("RX_FSK/git_version.h", "w")

fh.write("#define SPIFFS_MAJOR 2\n")
fh.write("#define SPIFFS_MINOR 17\n\n")
fh.write("extern const char *version_id;\n")
fh.write("extern const char *version_name;\n")
fh.close()

fc = open("RX_FSK/git_version.c", "w")

fc.write("#include \"git_version.h\"\n")
fc.write("\n")
fc.write("const char *version_name = \"rdzTTGOsonde\";\n")
fc.write("const char *version_id = \"Multi_ch-")
fc.write(git_versio[:-1])
fc.write("\";\n")
fc.close()

#########################################################################
# 
# Authors : Marc-Antoine Lacasse (malacasse@robotiq.com)
#           Eric Barnett (e.barnett@robotiq.com)
#           Oct. 2017
#
# Meta project
# 
#
#########################################################################

TEMPLATE = subdirs

# list of subprojects
SUBDIRS =  batotp \
           test \
           input \
           output

# qmake files
batotp.file = batotp/batotp.pro
test.file = test/test.pro
input.subdir = input
output.subdir = output

# dependencies
test.depends = batotp


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
SUBDIRS =  batotp\
           test

# qmake files
batotp.file = batotp/batotp.pro
test.file = test/test.pro

# dependencies
test.depends = batotp


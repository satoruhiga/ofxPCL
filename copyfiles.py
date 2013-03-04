#!/usr/bin/env python

import sys
import os
import shutil

if not sys.platform in ['darwin']:
	sys.exit(-1)

if len(sys.argv) != 2:
	print '$ copylibs.py [PATH_TO_YOUR_OF_PROJECT]'
	sys.exit(1)

target_path = os.path.abspath(sys.argv[1])
os.chdir(os.path.dirname(sys.argv[0]))

if not os.path.exists(target_path):
	print 'err: project dir not found'
	sys.exit(-1)

lib_base_path = 'libs/pcl/lib/osx'
lib_target_path = os.path.join(target_path, 'bin/data/pcl/lib')

print 'target_project =', target_path

if not os.path.exists(lib_target_path):
	os.makedirs(lib_target_path)

for i in os.listdir(lib_base_path):
	if not i.endswith('.dylib'): continue
	src = os.path.join(lib_base_path, i)
	dst = os.path.join(lib_target_path, i)
	print 'copy %s => %s' % (i, dst)

	shutil.copy(src, lib_target_path)

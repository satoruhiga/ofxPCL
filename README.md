# Setup instruction (OS X)

1. Get dependencies (88.2MB) and extract them to ofxPCL folder.

		$ curl -O http://structor.jp/dist/ofxpcl_deps.zip
		$ unzip ofxpcl_deps.zip


1. Add `HEADER_SEARCH_PATHS` to `Project.xcconfig`

		HEADER_SEARCH_PATHS = $(OF_CORE_HEADERS) $(OF_PATH)/addons/ofxPCL/deps/include $(OF_PATH)/addons/ofxPCL/deps/include/Eigen3 $(OF_PATH)/addons/ofxPCL/deps/include/vtk-5.8
	
1. Add library reference to Xcode project.
	
	Drag & Drop `deps/libs` folder.
	
	**DO NOT** add `deps/include` folder in Xcode project. 
	
1. Copy libraries to data folder

		$ cp -r PATH_TO_OFX_PCL/deps/libs/osx/pcl PATH_TO_YOUR_PROJECT/bin/data/pcl
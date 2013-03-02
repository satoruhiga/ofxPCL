# Setup instruction (OS X)

1. Get dependencies (96.9 MB) and extract them to ofxPCL folder.

		$ curl -O http://structor.jp/dist/ofxpcl_16_libs.zip
		$ unzip ofxpcl_16_libs.zip


1. Change `Project.xcconfig` like

		OFXPCL_PATH = $(OF_PATH)/addons/ofxPCL

		OFXPCL_OTHER_LDFLAGS = -L$(OFXPCL_PATH)/libs/pcl/lib/osx -lboost_chrono-mt -lboost_context-mt -lboost_date_time-mt -lboost_filesystem-mt -lboost_graph-mt -lboost_iostreams-mt -lboost_locale-mt -lboost_math_c99-mt -lboost_math_c99f-mt -lboost_math_c99l-mt -lboost_math_tr1-mt -lboost_math_tr1f-mt -lboost_math_tr1l-mt -lboost_prg_exec_monitor-mt -lboost_program_options-mt -lboost_python-mt -lboost_random-mt -lboost_regex-mt -lboost_serialization-mt -lboost_signals-mt -lboost_system-mt -lboost_thread-mt -lboost_timer-mt -lboost_unit_test_framework-mt -lboost_wave-mt -lboost_wserialization-mt -lflann -lflann_cpp -liconv -licudata -licui18n -licuio -licule -liculx -licutest -licutu -licuuc -lLSDyna -lpcl_common -lpcl_features -lpcl_filters -lpcl_geometry -lpcl_io -lpcl_io_ply -lpcl_kdtree -lpcl_keypoints -lpcl_octree -lpcl_registration -lpcl_sample_consensus -lpcl_search -lpcl_segmentation -lpcl_surface -lpcl_tracking -lpcl_visualization -lqhull -lvtkalglib -lvtkCharts -lvtkCommon -lvtkDICOMParser -lvtkexoIIc -lvtkexpat -lvtkFiltering -lvtkfreetype -lvtkftgl -lvtkGenericFiltering -lvtkGeovis -lvtkGraphics -lvtkhdf5 -lvtkhdf5_hl -lvtkHybrid -lvtkImaging -lvtkInfovis -lvtkIO -lvtkjpeg -lvtklibxml2 -lvtkmetaio -lvtkNetCDF -lvtkNetCDF_cxx -lvtkpng -lvtkproj4 -lvtkRendering -lvtksqlite -lvtksys -lvtktiff -lvtkverdict -lvtkViews -lvtkVolumeRendering -lvtkWidgets -lvtkzlib

		OFXPCL_HEADER_SEARCH_PATHS = $(OFXPCL_PATH)/libs/pcl/include/ $(OFXPCL_PATH)/libs/pcl/include/eigen3 $(OFXPCL_PATH)/libs/pcl/include/vtk-5.10 $(OFXPCL_PATH)/libs/pcl/include/pcl-1.6

		OTHER_LDFLAGS = $(OF_CORE_LIBS) $(OFXPCL_OTHER_LDFLAGS)
		HEADER_SEARCH_PATHS = $(OF_CORE_HEADERS) $(OFXPCL_HEADER_SEARCH_PATHS)

1. Add ofxPCL/src filder to Xcode project.

1. Copy libraries to data folder

		$ python copyfiles.py PATH_TO_YOUR_PROJECT
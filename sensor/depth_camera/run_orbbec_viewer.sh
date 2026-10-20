#!/bin/bash
# Launch Orbbec's official GUI viewer (OrbbecSDK v1.10.37) under WSLg.
cd "$(dirname "$0")/third_party/viewer/OrbbecViewer_v1.10.37_202607070725_linux_x64_release"
LD_LIBRARY_PATH=.:./lib:$LD_LIBRARY_PATH exec ./OrbbecViewer "$@"

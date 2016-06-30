#  Replace
#  
#     </BuildableProductRunnable>
#
#with
#  
#   </BuildableProductRunnable>
#      <EnvironmentVariables>
#         <EnvironmentVariable
#            key = "OPENCV_TEST_DATA_PATH"
#            value = "/Users/jaspershemilt/Developer/openCV_Dev/opencv_extra/testdata"
#            isEnabled = "YES">
#         </EnvironmentVariable>
#      </EnvironmentVariables>
#
#in 
#XCode project
# Generalise Later
#sudo ECHO "setting OPENCV_TEST_DATA_PATH " $1
#if [ $# -lt 1 ]
#  then
#    OPENCV_TEST_DATA_PATH=opt/local;
#    else
#    OPENCV_TEST_DATA_PATH=$1
#fi

perl -i -p -e 's/\s*<\/BuildableProductRunnable>$/      <\/BuildableProductRunnable>\
      <EnvironmentVariables>\
         <EnvironmentVariable\
            key = "OPENCV_TEST_DATA_PATH"\
            value = "\/Users\/jaspershemilt\/Developer\/openCV_Dev\/opencv_extra\/testdata"\
            isEnabled = "YES">\
         <\/EnvironmentVariable>\
      <\/EnvironmentVariables>/g;' osx/OpenCV.xcodeproj/xcuserdata/*.xcuserdatad/xcschemes/opencv_test_*.xcscheme
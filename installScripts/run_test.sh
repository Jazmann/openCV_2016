ECHO "Provide super user password please."
sudo ECHO "Hello superuser!"
export OPENCV_TEST_DATA_PATH=./opencv_extra/testdata
cd opencv
CURRENT_COMMIT=$(git rev-list -n1 --abbrev-commit HEAD);
cd ..

mkdir testResults_$CURRENT_COMMIT;
ATEST=$(find ./osx/*/*/opencv_test_*);
PTEST=$(find ./osx/*/*/opencv_perf_*);

for var in $ATEST;do
 afilename=$(basename "$var");
 ECHO "Found accuracy test $afilename";
done

for var in $PTEST;do
 pfilename=$(basename "$var");
 ECHO "Found performance test $pfilename";
done

MYREPLY="y";
read -p "Run accuracy tests : " -n 1 -r -t 5 MYREPLY
echo    # move to a new line
if [[ $MYREPLY =~ ^[Yy]$ ]] 
then
	for var in $ATEST
	do
	filename=$(basename "$var")
	MYREPLY="y";
	read -p "Run tests for $var : " -n 1 -r -t 5 MYREPLY
	echo    # (optional) move to a new line
	if [[ $MYREPLY =~ ^[Yy]$ ]]
	then
	  ECHO "Running test $filename";
	  $var &> ./testResults_$CURRENT_COMMIT/"$filename".log
	  grep -e FAILED ./testResults_$CURRENT_COMMIT/"$filename".log
	else 
	  ECHO "Not Running test $filename";
	fi
	done
fi

ECHO "Accuracy Tests Finished";

MYREPLY="y";
read -p "Run performance tests : " -n 1 -r -t 5 MYREPLY
echo    # move to a new line
if [[ $MYREPLY =~ ^[Yy]$ ]] 
then
	for var in $PTEST
	do
	filename=$(basename "$var")
	MYREPLY="y";
	read -p "Run tests for $var : " -n 1 -r -t 5 MYREPLY
	echo    # (optional) move to a new line
	if [[ $MYREPLY =~ ^[Yy]$ ]]
	then
	  ECHO "Running test $filename";
	  $var &> ./testResults_$CURRENT_COMMIT/"$filename".log
	  grep -e FAILED ./testResults_$CURRENT_COMMIT/"$filename".log
	else 
	  ECHO "Not Running test $filename";
	fi
    done
fi

ECHO "Performance Tests Finished";

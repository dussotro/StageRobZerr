VnaoPath=`pwd`/
if [ -z ${PYTHONPATH+x} ]
then
    #echo "PYTHONPATH is unset"
    export PYTHONPATH=${VnaoPath}/pynaoqi-python-2.7-2.1.4.13-linux64/
    export LD_LIBRARY_PATH="/home21/dussotro/Bureau/Stage/StageRobZerr/Test Cam/pynaoqi-python2.7-2.1.4.13-linux64"

else
    #echo "PYTHONPATH is set to '$PYTHONPATH'"
    export PYTHONPATH=${PYTHONPATH}:${VnaoPath}/pynaoqi-python-2.7-2.1.4.13-linux64/
    export LD_LIBRARY_PATH="/home21/dussotro/Bureau/Stage/StageRobZerr/Test Cam/pynaoqi-python2.7-2.1.4.13-linux64"
fi

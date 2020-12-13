if [ ! -d "./build" ]
then
	echo "Creating directory ./build"
	mkdir ./build
fi

cd ./build
cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug
make -j7
cd ..

# compile and generate executable main file. 
all:
	 g++ main.cpp -o main.exe  
	 ./main.exe

# removes any .exe files from the workspace folder using rm -rf command which is apparently for Mac but seems to work for Windows too. 
clean: 
	rm -rf  *.exe 
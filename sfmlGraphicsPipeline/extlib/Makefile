UNAME := $(shell uname)

all: build_freetype build_tinyobjloader build_glew build_sfml

build_freetype:
ifeq ($(UNAME), Linux)
	cd freetype-2.5.5/ ; mkdir -p build ; cd build/ && cmake ../ && make -j 8
endif
ifeq ($(UNAME), Windows)
	cd freetype-2.5.5/ && if not exist "build" mkdir build && cd build/ && cmake -G "MinGW Makefiles" ../ && make CC=g++ -j 8
endif
	
build_tinyobjloader:
ifeq ($(UNAME), Linux)
	cd tinyobjloader/ ; mkdir -p build ; cd build/ && cmake ../ && make -j 8
endif
ifeq ($(UNAME), Windows)
	cd tinyobjloader/ && if not exist "build" mkdir build && cd build/ && cmake -G "MinGW Makefiles" ../ && make CC=g++ -j 8
endif

build_glew:
ifeq ($(UNAME), Windows)
	cd glew-1.13.0/ && if not exist "lib" mkdir lib && gcc -DGLEW_NO_GLU -O2 -Wall -W -Iinclude  -DGLEW_BUILD -o src/glew.o -c src/glew.c && gcc -shared -Wl,-soname,libglew32.dll -Wl,--out-implib,lib/libglew32.dll.a    -o lib/glew32.dll src/glew.o -L/mingw/lib -lglu32 -lopengl32 -lgdi32 -luser32 -lkernel32 && ar cr lib/libglew32.a src/glew.o && gcc -DGLEW_NO_GLU -DGLEW_MX -O2 -Wall -W -Iinclude  -DGLEW_BUILD -o src/glew.mx.o -c src/glew.c && gcc -shared -Wl,-soname,libglew32mx.dll -Wl,--out-implib,lib/libglew32mx.dll.a -o lib/glew32mx.dll src/glew.mx.o -L/mingw/lib -lglu32 -lopengl32 -lgdi32 -luser32 -lkernel32 && ar cr lib/libglew32mx.a src/glew.mx.o
endif

build_sfml:
ifeq ($(UNAME), Windows)
	cd SFML-2.5.1/ && if not exist "build" mkdir build && cd build/ && cmake -G "MinGW Makefiles" ../ && make CC=g++ -j 8
endif

clean: clean_freetype clean_tinyobjloader clean_glew clean_sfml

clean_freetype:
ifeq ($(UNAME), Linux)
	cd freetype-2.5.5/ && rm -r build/
endif
ifeq ($(UNAME), Windows)
	cd freetype-2.5.5/ && if exist "build" rmdir /s /q build
endif	

clean_tinyobjloader:
ifeq ($(UNAME), Linux)
	cd tinyobjloader && rm -rf build/
endif
ifeq ($(UNAME), Windows)
	cd tinyobjloader/ && if exist "build" rmdir /s /q build
endif

clean_glew:
ifeq ($(UNAME), Windows)
	cd glew-1.13.0 && if exist "build" rmdir /s /q build
endif	

clean_sfml:
ifeq ($(UNAME), Linux)
	cd SFML-2.5.1/ && rm -rf build/
endif
ifeq ($(UNAME), Windows)
	cd SFML-2.5.1/ && if exist "lib" rmdir /s /q lib
endif
###########################################################################
## Makefile generated for component 'mpcController'. 
## 
## Makefile     : mpcController_rtw.mk
## Generated on : Sat Oct 24 11:16:27 2026
## Final product: ./mpcController.a
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# MODELLIB                Static library target

PRODUCT_NAME              = mpcController
MAKEFILE                  = mpcController_rtw.mk
MATLAB_ROOT               = /Applications/MATLAB_R2023b.app
MATLAB_BIN                = /Applications/MATLAB_R2023b.app/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/maca64
START_DIR                 = /Users/pappistvan/Doksik/szakdolgozat/matlab_implementations/MPC
TGT_FCN_LIB               = ISO_C++11
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 
RELATIVE_PATH_TO_ANCHOR   = ../../..
C_STANDARD_OPTS           = -fno-common -fexceptions
CPP_STANDARD_OPTS         = -std=c++14 -fno-common -fexceptions
MODELLIB                  = mpcController.a

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          Xcode with Clang | gmake (64-bit Mac)
# Supported Version(s):    
# ToolchainInfo Version:   2023b
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# C_STANDARD_OPTS
# CPP_STANDARD_OPTS

#-----------
# MACROS
#-----------

ARCHS                               = arm64
XCODE_SDK_VER                       = $(shell perl $(MATLAB_ROOT)/rtw/c/tools/macsdkver.pl)
XCODE_SDK                           = MacOSX$(XCODE_SDK_VER).sdk
XCODE_DEVEL_DIR                     = $(shell xcode-select -print-path)
XCODE_SDK_ROOT                      = $(XCODE_DEVEL_DIR)/Platforms/MacOSX.platform/Developer/SDKs/$(XCODE_SDK)
XCODE_LD_VERSION                    = $(shell xcrun ld -v 2>&1 >/dev/null | head -n 1)
XCODE_LD_VERSION_IS_DYLD            = $(findstring dyld,$(XCODE_LD_VERSION))
XCODE_DYLD_NO_WARN_DUPLICATE_LDFLAG = -Wl,-no_warn_duplicate_libraries
NO_WARN_DUPLICATE_LIBRARIES         = $(if $(XCODE_LD_VERSION_IS_DYLD),$(XCODE_DYLD_NO_WARN_DUPLICATE_LDFLAG),)

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = 

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: Clang C Compiler
CC = xcrun clang

# Linker: Clang Linker
LD = xcrun clang++

# C++ Compiler: Clang C++ Compiler
CPP = xcrun clang++

# C++ Linker: Clang C++ Linker
CPP_LD = xcrun clang++

# Archiver: Clang Archiver
AR = xcrun ar

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)/mex"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: GMAKE Utility
MAKE_PATH = %MATLAB%/bin/maca64
MAKE = "$(MAKE_PATH)/gmake"


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  = @rm -f
ECHO                = @echo
MV                  = @mv
RUN                 =

#--------------------------------------
# "Faster Runs" Build Configuration
#--------------------------------------

ARFLAGS              = ruvs
CFLAGS               = -c -isysroot $(XCODE_SDK_ROOT) -arch $(ARCHS) $(C_STANDARD_OPTS) -mmacosx-version-min=11.0 \
                       -O3 -ffp-contract=off
CPPFLAGS             = -c -isysroot $(XCODE_SDK_ROOT) -arch $(ARCHS) $(CPP_STANDARD_OPTS) -mmacosx-version-min=11.0 \
                       -O3 -ffp-contract=off
CPP_LDFLAGS          = -arch $(ARCHS) -isysroot $(XCODE_SDK_ROOT) -Wl,-rpath,@executable_path -Wl,-rpath,@executable_path/$(RELATIVE_PATH_TO_ANCHOR) $(NO_WARN_DUPLICATE_LIBRARIES)
CPP_SHAREDLIB_LDFLAGS  = -arch $(ARCHS)  \
                         -dynamiclib -install_name @rpath/$(notdir $(PRODUCT)) -isysroot $(XCODE_SDK_ROOT) \
                         -Wl,$(LD_NAMESPACE) $(LD_UNDEFS) \
                         $(NO_WARN_DUPLICATE_LIBRARIES)
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              = -arch $(ARCHS) -isysroot $(XCODE_SDK_ROOT) -Wl,-rpath,@executable_path -Wl,-rpath,@executable_path/$(RELATIVE_PATH_TO_ANCHOR) $(NO_WARN_DUPLICATE_LIBRARIES)
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    = -arch $(ARCHS)  \
                       -dynamiclib -install_name @rpath/$(notdir $(PRODUCT)) -isysroot $(XCODE_SDK_ROOT) \
                       -Wl,$(LD_NAMESPACE) $(LD_UNDEFS) \
                       $(NO_WARN_DUPLICATE_LIBRARIES)



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = ./mpcController.a
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I$(START_DIR)/codegen/lib/mpcController -I$(START_DIR) -I$(MATLAB_ROOT)/extern/include

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_CUSTOM = 
DEFINES_STANDARD = -DMODEL=mpcController

DEFINES = $(DEFINES_CUSTOM) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)/codegen/lib/mpcController/rt_nonfinite.cpp $(START_DIR)/codegen/lib/mpcController/rtGetNaN.cpp $(START_DIR)/codegen/lib/mpcController/rtGetInf.cpp $(START_DIR)/codegen/lib/mpcController/mpcController_initialize.cpp $(START_DIR)/codegen/lib/mpcController/mpcController_terminate.cpp $(START_DIR)/codegen/lib/mpcController/mpcController.cpp $(START_DIR)/codegen/lib/mpcController/factoryConstruct.cpp $(START_DIR)/codegen/lib/mpcController/factoryConstruct1.cpp $(START_DIR)/codegen/lib/mpcController/xnrm2.cpp $(START_DIR)/codegen/lib/mpcController/computeComplError.cpp $(START_DIR)/codegen/lib/mpcController/fmincon.cpp $(START_DIR)/codegen/lib/mpcController/compressBounds.cpp $(START_DIR)/codegen/lib/mpcController/computeFiniteDifferences.cpp $(START_DIR)/codegen/lib/mpcController/setProblemType.cpp $(START_DIR)/codegen/lib/mpcController/driver.cpp $(START_DIR)/codegen/lib/mpcController/test_exit.cpp $(START_DIR)/codegen/lib/mpcController/xgeqp3.cpp $(START_DIR)/codegen/lib/mpcController/xzgeqp3.cpp $(START_DIR)/codegen/lib/mpcController/xzlarfg.cpp $(START_DIR)/codegen/lib/mpcController/xzlarf.cpp $(START_DIR)/codegen/lib/mpcController/xgerc.cpp $(START_DIR)/codegen/lib/mpcController/computeQ_.cpp $(START_DIR)/codegen/lib/mpcController/sortLambdaQP.cpp $(START_DIR)/codegen/lib/mpcController/step.cpp $(START_DIR)/codegen/lib/mpcController/driver1.cpp $(START_DIR)/codegen/lib/mpcController/PresolveWorkingSet.cpp $(START_DIR)/codegen/lib/mpcController/countsort.cpp $(START_DIR)/codegen/lib/mpcController/removeConstr.cpp $(START_DIR)/codegen/lib/mpcController/feasibleX0ForWorkingSet.cpp $(START_DIR)/codegen/lib/mpcController/factorQR.cpp $(START_DIR)/codegen/lib/mpcController/maxConstraintViolation.cpp $(START_DIR)/codegen/lib/mpcController/computeFval.cpp $(START_DIR)/codegen/lib/mpcController/linearForm_.cpp $(START_DIR)/codegen/lib/mpcController/iterate.cpp $(START_DIR)/codegen/lib/mpcController/computeGrad_StoreHx.cpp $(START_DIR)/codegen/lib/mpcController/computeFval_ReuseHx.cpp $(START_DIR)/codegen/lib/mpcController/xrotg.cpp $(START_DIR)/codegen/lib/mpcController/deleteColMoveEnd.cpp $(START_DIR)/codegen/lib/mpcController/compute_deltax.cpp $(START_DIR)/codegen/lib/mpcController/xpotrf.cpp $(START_DIR)/codegen/lib/mpcController/factor.cpp $(START_DIR)/codegen/lib/mpcController/solve.cpp $(START_DIR)/codegen/lib/mpcController/xgemm.cpp $(START_DIR)/codegen/lib/mpcController/addBoundToActiveSetMatrix_.cpp $(START_DIR)/codegen/lib/mpcController/relaxed.cpp $(START_DIR)/codegen/lib/mpcController/BFGSUpdate.cpp $(START_DIR)/codegen/lib/mpcController/feasibleratiotest.cpp $(START_DIR)/codegen/lib/mpcController/partialColLDL3_.cpp $(START_DIR)/codegen/lib/mpcController/fullColLDL2_.cpp

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = rt_nonfinite.o rtGetNaN.o rtGetInf.o mpcController_initialize.o mpcController_terminate.o mpcController.o factoryConstruct.o factoryConstruct1.o xnrm2.o computeComplError.o fmincon.o compressBounds.o computeFiniteDifferences.o setProblemType.o driver.o test_exit.o xgeqp3.o xzgeqp3.o xzlarfg.o xzlarf.o xgerc.o computeQ_.o sortLambdaQP.o step.o driver1.o PresolveWorkingSet.o countsort.o removeConstr.o feasibleX0ForWorkingSet.o factorQR.o maxConstraintViolation.o computeFval.o linearForm_.o iterate.o computeGrad_StoreHx.o computeFval_ReuseHx.o xrotg.o deleteColMoveEnd.o compute_deltax.o xpotrf.o factor.o solve.o xgemm.o addBoundToActiveSetMatrix_.o relaxed.o BFGSUpdate.o feasibleratiotest.o partialColLDL3_.o fullColLDL2_.o

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS =  -lm -lstdc++

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += $(CPPFLAGS_BASIC)

###########################################################################
## INLINED COMMANDS
###########################################################################

###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute


all : build
	@echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


prebuild : 


download : $(PRODUCT)


execute : download


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS)
	@echo "### Creating static library "$(PRODUCT)" ..."
	$(AR) $(ARFLAGS)  $(PRODUCT) $(OBJS)
	@echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.CC
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.cp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.CPP
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.c++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.C++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : %.CXX
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.CC
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.CPP
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.C++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.CXX
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/lib/mpcController/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/lib/mpcController/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/lib/mpcController/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/lib/mpcController/%.CC
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/lib/mpcController/%.cp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/lib/mpcController/%.CPP
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/lib/mpcController/%.c++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/lib/mpcController/%.C++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/lib/mpcController/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/codegen/lib/mpcController/%.CXX
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cc
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.CC
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.CPP
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.c++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.C++
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cxx
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.CXX
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rt_nonfinite.o : $(START_DIR)/codegen/lib/mpcController/rt_nonfinite.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rtGetNaN.o : $(START_DIR)/codegen/lib/mpcController/rtGetNaN.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


rtGetInf.o : $(START_DIR)/codegen/lib/mpcController/rtGetInf.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


mpcController_initialize.o : $(START_DIR)/codegen/lib/mpcController/mpcController_initialize.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


mpcController_terminate.o : $(START_DIR)/codegen/lib/mpcController/mpcController_terminate.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


mpcController.o : $(START_DIR)/codegen/lib/mpcController/mpcController.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


factoryConstruct.o : $(START_DIR)/codegen/lib/mpcController/factoryConstruct.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


factoryConstruct1.o : $(START_DIR)/codegen/lib/mpcController/factoryConstruct1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xnrm2.o : $(START_DIR)/codegen/lib/mpcController/xnrm2.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


computeComplError.o : $(START_DIR)/codegen/lib/mpcController/computeComplError.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


fmincon.o : $(START_DIR)/codegen/lib/mpcController/fmincon.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


compressBounds.o : $(START_DIR)/codegen/lib/mpcController/compressBounds.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


computeFiniteDifferences.o : $(START_DIR)/codegen/lib/mpcController/computeFiniteDifferences.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


setProblemType.o : $(START_DIR)/codegen/lib/mpcController/setProblemType.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


driver.o : $(START_DIR)/codegen/lib/mpcController/driver.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


test_exit.o : $(START_DIR)/codegen/lib/mpcController/test_exit.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xgeqp3.o : $(START_DIR)/codegen/lib/mpcController/xgeqp3.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xzgeqp3.o : $(START_DIR)/codegen/lib/mpcController/xzgeqp3.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xzlarfg.o : $(START_DIR)/codegen/lib/mpcController/xzlarfg.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xzlarf.o : $(START_DIR)/codegen/lib/mpcController/xzlarf.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xgerc.o : $(START_DIR)/codegen/lib/mpcController/xgerc.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


computeQ_.o : $(START_DIR)/codegen/lib/mpcController/computeQ_.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


sortLambdaQP.o : $(START_DIR)/codegen/lib/mpcController/sortLambdaQP.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


step.o : $(START_DIR)/codegen/lib/mpcController/step.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


driver1.o : $(START_DIR)/codegen/lib/mpcController/driver1.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


PresolveWorkingSet.o : $(START_DIR)/codegen/lib/mpcController/PresolveWorkingSet.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


countsort.o : $(START_DIR)/codegen/lib/mpcController/countsort.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


removeConstr.o : $(START_DIR)/codegen/lib/mpcController/removeConstr.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


feasibleX0ForWorkingSet.o : $(START_DIR)/codegen/lib/mpcController/feasibleX0ForWorkingSet.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


factorQR.o : $(START_DIR)/codegen/lib/mpcController/factorQR.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


maxConstraintViolation.o : $(START_DIR)/codegen/lib/mpcController/maxConstraintViolation.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


computeFval.o : $(START_DIR)/codegen/lib/mpcController/computeFval.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


linearForm_.o : $(START_DIR)/codegen/lib/mpcController/linearForm_.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


iterate.o : $(START_DIR)/codegen/lib/mpcController/iterate.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


computeGrad_StoreHx.o : $(START_DIR)/codegen/lib/mpcController/computeGrad_StoreHx.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


computeFval_ReuseHx.o : $(START_DIR)/codegen/lib/mpcController/computeFval_ReuseHx.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xrotg.o : $(START_DIR)/codegen/lib/mpcController/xrotg.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


deleteColMoveEnd.o : $(START_DIR)/codegen/lib/mpcController/deleteColMoveEnd.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


compute_deltax.o : $(START_DIR)/codegen/lib/mpcController/compute_deltax.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xpotrf.o : $(START_DIR)/codegen/lib/mpcController/xpotrf.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


factor.o : $(START_DIR)/codegen/lib/mpcController/factor.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


solve.o : $(START_DIR)/codegen/lib/mpcController/solve.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


xgemm.o : $(START_DIR)/codegen/lib/mpcController/xgemm.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


addBoundToActiveSetMatrix_.o : $(START_DIR)/codegen/lib/mpcController/addBoundToActiveSetMatrix_.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


relaxed.o : $(START_DIR)/codegen/lib/mpcController/relaxed.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


BFGSUpdate.o : $(START_DIR)/codegen/lib/mpcController/BFGSUpdate.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


feasibleratiotest.o : $(START_DIR)/codegen/lib/mpcController/feasibleratiotest.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


partialColLDL3_.o : $(START_DIR)/codegen/lib/mpcController/partialColLDL3_.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


fullColLDL2_.o : $(START_DIR)/codegen/lib/mpcController/fullColLDL2_.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@echo "### PRODUCT = $(PRODUCT)"
	@echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@echo "### BUILD_TYPE = $(BUILD_TYPE)"
	@echo "### INCLUDES = $(INCLUDES)"
	@echo "### DEFINES = $(DEFINES)"
	@echo "### ALL_SRCS = $(ALL_SRCS)"
	@echo "### ALL_OBJS = $(ALL_OBJS)"
	@echo "### LIBS = $(LIBS)"
	@echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	@echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@echo "### CFLAGS = $(CFLAGS)"
	@echo "### LDFLAGS = $(LDFLAGS)"
	@echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@echo "### CPPFLAGS = $(CPPFLAGS)"
	@echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	@echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	@echo "### ARFLAGS = $(ARFLAGS)"
	@echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	@echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	@echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	@echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	@echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	@echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	@echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files ..."
	$(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(ECHO) "### Deleted all derived files."



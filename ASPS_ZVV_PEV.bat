@rem *********************
@rem Makefile for c6657
@rem *********************


@rem *** Path ***
@set PEOS_PATH="C:\Program Files (x86)\Myway Plus\PE-ViewX\pe-viewx\PEOS\c6657\3_12"


@rem *** Target File ***
@set TARGET=ASPS_ZVV_PEV
@set GOAL=target.btbl


@rem *** Tools Name ***
@set COMPILER_DIR="C:\ti\c6000_7.4.24\bin"
@set CC=%COMPILER_DIR%\cl6x
@set LD=%COMPILER_DIR%\cl6x
@set HEX=%COMPILER_DIR%\hex6x
@set NM=%COMPILER_DIR%\nm6x
@set MAKE_DEF=make_def
@set MAKE_FUNCDEF=make_funcdef
@set MAKE_SBL=c6657_make_sbl
@set MAKE_FUNCSBL=c6657_make_funcsbl
@set MAKE_TYP=make_typ


@rem *** Objects & Libraries ***
@set OBJS=^
	ASPS_ZVV_PEV.obj ^
	Table_Reference.obj ^
	DSPBoard.obj ^
	InductionMotor.obj ^
	LowPassFilter.obj ^
	Common.obj ^
	Vector.obj ^
	Inverter.obj ^
	Integrator.obj

@set LIBS="C:\Program Files (x86)\Myway Plus\PE-ViewX\pe-viewx\PEOS\c6657\3_12\lib\Main.obj" ^
 "C:\Program Files (x86)\Myway Plus\PE-ViewX\pe-viewx\PEOS\c6657\3_12\lib\mwio4.lib" ^
 "C:\ti\mathlib_c66x_3_0_1_1\lib\mathlib.ae66" ^
 "C:\ti\c6000_7.4.24\bin\..\lib\rts6600_elf.lib" ^
 "C:\ti\pdk_C6657_1_1_2_6\packages\ti\csl\lib\ti.csl.ae66" ^
 "C:\ti\pdk_C6657_1_1_2_6\packages\ti\csl\lib\ti.csl.intc.ae66"


@rem *** Others ***
@set SYMBOL=^
	ASPS_ZVV_PEV.@sbl ^
	 + Table_Reference.@sbl ^
	 + DSPBoard.@sbl ^
	 + InductionMotor.@sbl ^
	 + LowPassFilter.@sbl ^
	 + Common.@sbl ^
	 + Vector.@sbl ^
	 + Inverter.@sbl ^
	 + Integrator.@sbl ^
 

@set SYMBOL_CLEAR=^
	ASPS_ZVV_PEV.@sbl ^
	Table_Reference.@sbl ^
	DSPBoard.@sbl ^
	InductionMotor.@sbl ^
	LowPassFilter.@sbl ^
	Common.@sbl ^
	Vector.@sbl ^
	Inverter.@sbl ^
	Integrator.@sbl ^
 

@set FUNCSYMBOL=^
	ASPS_ZVV_PEV.@funcsbl ^
	 + Table_Reference.@funcsbl ^
	 + DSPBoard.@funcsbl ^
	 + InductionMotor.@funcsbl ^
	 + LowPassFilter.@funcsbl ^
	 + Common.@funcsbl ^
	 + Vector.@funcsbl ^
	 + Inverter.@funcsbl ^
	 + Integrator.@funcsbl ^
 

@set FUNCSYMBOL_CLEAR=^
	ASPS_ZVV_PEV.@funcsbl ^
	Table_Reference.@funcsbl ^
	DSPBoard.@funcsbl ^
	InductionMotor.@funcsbl ^
	LowPassFilter.@funcsbl ^
	Common.@funcsbl ^
	Vector.@funcsbl ^
	Inverter.@funcsbl ^
	Integrator.@funcsbl ^
 


@rem *** Flags ***
@set BASE_CFLAGS=-mv6600 --display_error_number --preproc_with_compile --diag_warning=225 --abi=eabi -O2 -i"C:/Users/ohlab/Documents/Hayashi/ASPS_ZVV_PEV" -i"C:/Program Files (x86)/Myway Plus/PE-ViewX/pe-viewx/PEOS/c6657/3_12/inc" -i"C:/ti/c6000_7.4.24/bin/../include" -i"C:/ti/pdk_C6657_1_1_2_6/packages/ti/csl" -i"C:/ti/mathlib_c66x_3_0_1_1/inc" -i"C:/ti/mathlib_c66x_3_0_1_1/packages" -i"C:/ti/pdk_C6657_1_1_2_6/packages/ti/csl/../.."
@set CFLAGS=%BASE_CFLAGS% -k
@set ASMFLAGS=%BASE_CFLAGS%
@set LDFLAGS=--run_linker --rom_model --map_file=%TARGET%.map -l=%LIBS% -l=%TARGET%.cmd --zero_init=off 
@set HEXFLAGS=-m3 --memwidth=8


@rem *** Delete ***
@echo off
del %OBJS% %TARGET%.out target.btbl %TARGET%.typ %TARGET%.map %TARGET%.def %TARGET%.funcdef %TARGET%.tmp@@ %TARGET%.functmp@@ %TARGET%.all_sym %SYMBOL_CLEAR% %FUNCSYMBOL_CLEAR% 2> nul
@echo on


@rem *** Compile ***
%CC% %CFLAGS% ASPS_ZVV_PEV.c
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_SBL% ASPS_ZVV_PEV.asm ASPS_ZVV_PEV.@sbl
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_FUNCSBL% ASPS_ZVV_PEV.asm ASPS_ZVV_PEV.@funcsbl
@echo off & if errorlevel 1 goto ERR
del ASPS_ZVV_PEV.asm 2> nul
@echo on


%CC% %CFLAGS% lib\Table_Reference.c
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_SBL% Table_Reference.asm Table_Reference.@sbl
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_FUNCSBL% Table_Reference.asm Table_Reference.@funcsbl
@echo off & if errorlevel 1 goto ERR
del Table_Reference.asm 2> nul
@echo on


%CC% %CFLAGS% lib\DSPBoard.c
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_SBL% DSPBoard.asm DSPBoard.@sbl
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_FUNCSBL% DSPBoard.asm DSPBoard.@funcsbl
@echo off & if errorlevel 1 goto ERR
del DSPBoard.asm 2> nul
@echo on


%CC% %CFLAGS% lib\InductionMotor.c
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_SBL% InductionMotor.asm InductionMotor.@sbl
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_FUNCSBL% InductionMotor.asm InductionMotor.@funcsbl
@echo off & if errorlevel 1 goto ERR
del InductionMotor.asm 2> nul
@echo on


%CC% %CFLAGS% lib\LowPassFilter.c
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_SBL% LowPassFilter.asm LowPassFilter.@sbl
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_FUNCSBL% LowPassFilter.asm LowPassFilter.@funcsbl
@echo off & if errorlevel 1 goto ERR
del LowPassFilter.asm 2> nul
@echo on


%CC% %CFLAGS% lib\Common.c
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_SBL% Common.asm Common.@sbl
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_FUNCSBL% Common.asm Common.@funcsbl
@echo off & if errorlevel 1 goto ERR
del Common.asm 2> nul
@echo on


%CC% %CFLAGS% lib\Vector.c
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_SBL% Vector.asm Vector.@sbl
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_FUNCSBL% Vector.asm Vector.@funcsbl
@echo off & if errorlevel 1 goto ERR
del Vector.asm 2> nul
@echo on


%CC% %CFLAGS% lib\Inverter.c
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_SBL% Inverter.asm Inverter.@sbl
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_FUNCSBL% Inverter.asm Inverter.@funcsbl
@echo off & if errorlevel 1 goto ERR
del Inverter.asm 2> nul
@echo on


%CC% %CFLAGS% lib\Integrator.c
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_SBL% Integrator.asm Integrator.@sbl
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_FUNCSBL% Integrator.asm Integrator.@funcsbl
@echo off & if errorlevel 1 goto ERR
del Integrator.asm 2> nul
@echo on


@rem *** Link ***
@echo off
copy %SYMBOL%  %TARGET%.tmp@@ > nul
@echo on
@echo off
copy %FUNCSYMBOL%  %TARGET%.functmp@@ > nul
@echo on
%MAKE_TYP% %TARGET%.tmp@@ ASPS_ZVV_PEV.typ
@echo off & if errorlevel 1 goto ERR
@echo on
%LD% -o %OBJS% %LDFLAGS% --output_file=%TARGET%.out
@echo off & if errorlevel 1 goto ERR
@echo on


@rem *** Generates *.def ***
%HEX% -order L linker_image.rmd %TARGET%.out
@echo off & if errorlevel 1 goto ERR
@echo on
%NM% -a %TARGET%.out > %TARGET%.all_sym
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_DEF% %TARGET%.typ %PEOS_PATH%\config\Type.cfg %TARGET%.all_sym
@echo off & if errorlevel 1 goto ERR
@echo on
%MAKE_FUNCDEF% %TARGET%.functmp@@ %TARGET%.all_sym
@echo off & if errorlevel 1 goto ERR
del %OBJS% %TARGET%.typ %TARGET%.tmp@@ %TARGET%.functmp@@ %TARGET%.all_sym %SYMBOL_CLEAR% %FUNCSYMBOL_CLEAR% 2> nul


@echo off
exit 0


@rem ** Error Process ***
:ERR
@echo off
del %OBJS% %TARGET%.typ %TARGET%.tmp@@ %TARGET%.functmp@@ %TARGET%.all_sym %SYMBOL_CLEAR% %FUNCSYMBOL_CLEAR% 2> nul
exit 1

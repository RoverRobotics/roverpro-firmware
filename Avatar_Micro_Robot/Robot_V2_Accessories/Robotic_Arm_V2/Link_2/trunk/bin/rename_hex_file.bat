setlocal EnableDelayedExpansion
rem @echo off
echo .
echo .
echo .


set date_name=%DATE:~-4%_%DATE:~4,2%_%DATE:~7,2%
set date_name_short=%DATE:~-4%%DATE:~4,2%%DATE:~7,2%

set time_name=%TIME:~0,2%_%TIME:~3,2%
set time_name_short=%TIME:~0,2%%TIME:~3,2%
set file_date=%DATE:~4,2%-%DATE:~7,2%-%DATE:~-4%

if "%time_name:~0,1%"==" " (
echo Correcting for space at beginning of time
set time_name=%TIME:~1,1%_%TIME:~3,2%_%TIME:~6,2%
)

set file_name=%date_name%_%time_name%


REM set /P svn_revision="Enter SVN revision number: "



	
	for /f "tokens=1  delims=" %%a IN ('dir /tw *.hex') do (
		rem echo next %%a
		set /a i=!i!+1
		if !i! == 4 (
			set file_time=%%a
			set date_string=!file_time:~0,10!
			set time_string=!file_time:~12,8!
			echo !date_string!
			echo !time_string!
		)
	)
	echo !date_string!
	echo !time_string!
	set month_string=!date_string:~0,2!
	set day_string=!date_string:~3,2!
	set year_string=!date_string:~6,4!
	set hour_string=!time_string:~0,2!
	set minute_string=!time_string:~3,2!
	set am_pm=!time_string:~6,2!

	REM set backup_folder=backup_!year_string!_!month_string!_!day_string!_file_time_!hour_string!_!minute_string!_!am_pm!_backed_up_%file_name%
	REM echo %backup_folder%
	REM mkdir %backup_folder%
	mkdir releases
	mkdir releases\"%date_name%_%time_name%"\


	copy firmware.hex releases\"%date_name%_%time_name%"\firmware_compiled_!year_string!_!month_string!_!day_string!_time_!hour_string!_!minute_string!_!am_pm!_(%date_name_short%.%time_name_short%).hex


)

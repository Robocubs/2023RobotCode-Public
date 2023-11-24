taskkill /im chrome.exe /f /t
taskkill /im python3.9.exe /f /t
taskkill /im DriverStation.exe /f /t
taskkill /im javaw.exe /f /t
cd\Users\robocubs\2023RobotCode-Private\src\operatorserver
start /min python3.9 -m pynetworktables2js --dashboard --team 1701
cd\Users\robocubs\AppData\Local\Google\Chrome\Application
start chrome.exe --start-fullscreen --hide-crash-restore-bubble http://localhost:8888/
cd\progra~2\frcdri~1\
start DriverStation.exe
cd C:\fusion

:: pip install virtualenv

:: virtualenv -p  C:\Users\Admin\AppData\Local\Autodesk\webdeploy\production\22e664d43fce268ac9bf1fa1cda69e4a9585c997\Python\python.exe py39_fusion

call py39_fusion\Scripts\activate

pip install numpy
pip install openpyxl
pip install pandas

pip list

deactivate
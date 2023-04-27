cd C:\fusion

:: pip install virtualenv

virtualenv -p  C:\Users\Admin\AppData\Local\Autodesk\webdeploy\production\d2926c0e17450af0d0d926a3d6da4c873b8a2007\Python\python.exe py39_fusion

call py39_fusion\Scripts\activate

python.exe -m pip install --upgrade pip

pip install numpy
pip install openpyxl
pip install sympy
pip install scipy
pip list

deactivate
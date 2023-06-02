cd C:\fusion

:: pip install virtualenv

:: virtualenv -p  C:\Users\Admin\AppData\Local\Autodesk\webdeploy\production\a025d29cc566c591e9a766a22ab936c55abdd11a\Python\python.exe py39_fusion

call py39_fusion\Scripts\activate

python -m pip install --upgrade pip

pip install numpy
pip install sympy
pip install openpyxl
pip install cupy-cuda12x
pip install scipy

pip list

deactivate
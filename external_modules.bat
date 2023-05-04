cd C:\fusion

:: pip install virtualenv

virtualenv -p  C:\Users\Admin\AppData\Local\Autodesk\webdeploy\production\b4885f4229f39fee5ad2bce82f309e671e5c9ccd\Python\python.exe py39_fusion

call py39_fusion\Scripts\activate

python.exe -m pip install --upgrade pip

pip install notebook
pip install numpy
pip install openpyxl
pip install sympy
pip install scipy
pip list

deactivate
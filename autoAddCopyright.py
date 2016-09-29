# Copyright (c) 2016-2017 EEC193 Senior Design at UCDavis

# This program is free software: you can redistribute it and/or modify it under the terms of the Lesser GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.

# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# This program will add copyright:LGPL license to every source file in UCD-UAV program

import os.path

root_dir = os.getcwd()
declaration=['Copyright (c) 2016-2017 EEC193 Senior Design at UCDavis',
'This program is free software: you can redistribute it and/or modify it under the terms of the Lesser GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or(at your option) any later version.',
'The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.',
'THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.']
comment={'py':'#','c':'//','cpp':'//'}

def add_license(x, dir_name, files):
    #traverse the files
    for file in files:
        file_name = dir_name+'/'+file
        with open(file_name,'r+') as f:
            content = f.read()
            f.seek(0, 0)
            for line in declaration:
                if (file.split('.')[1] in comment.keys()):
                    f.write(comment[file.split('.')[1]]+line+ '\n')
            f.write(content)

        #add declaration

os.path.walk('./', add_license, 0)
# add_license(0,'.',['sample.py'])
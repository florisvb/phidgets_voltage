import subprocess

def get_serial_numbers_for_idvend_idprod_name(idvend, idprod, name):
    '''
    extract the serial numbers from lsusb, for example: '06c2', '0037', 'Phidgets' will find all devices matching 06c2:0037 Phidgets
    '''
    search_string = str(idvend) + ':' + str(idprod) + ' ' + name
    search_string = search_string.encode('utf-8')
    print(search_string)

    v = subprocess.Popen(["lsusb", "-v"], stdout=subprocess.PIPE)
    data = v.communicate()[0]
    lines = data.split(b'\n')

    phidgets_lines = []
    for l, line in enumerate(lines):
        if search_string in line:
            phidgets_lines.append(l)

    serial_numbers = []
    for l in phidgets_lines:
        phidgets_info = lines[l:l+20]
        for line in phidgets_info:
            if b'iSerial' in line:
                print(line)
                serial_numbers.append(int(line.split(b' ')[-1]))

    return serial_numbers

if __name__ == '__main__':

    serials = get_serial_numbers_for_idvend_idprod_name('06c2', '0037', 'Phidgets')
    print(serials)
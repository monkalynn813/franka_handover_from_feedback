import http.client

def read_to_file():
    save_to_path='/home/jingyan/Documents/handover_franka/handover_control_ws/src/handover_controller/sample/sample.csv'

    connection = http.client.HTTPConnection('192.168.0.46', 8080)
    connection.request("GET", "/data.csv")

    response = connection.getresponse().read().decode('utf-8').split(',')
    row=''
    for i in range(len(response)-1):

        row += response[i]
        row += ','
    row+= response[-1]
    with open(save_to_path,'w') as f:
        f.write(row)
    
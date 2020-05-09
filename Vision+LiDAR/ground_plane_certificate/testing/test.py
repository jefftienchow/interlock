import yaml
import system as system
import basic_plot as plt
import time
import numpy

def get_data(f='lidar_data.yaml'):
    with open(f, 'r') as stream:
        rows_to_data_lists = yaml.safe_load(stream)

    flat_pts = []
    pts_to_row = {}
    for row, data_list in rows_to_data_lists.items():
        flat_pts.extend(data_list)
        for pt in data_list:
            pts_to_row[tuple(pt)] = row

    flat_pts = numpy.array(flat_pts)

    print('done grabbing data')
    return flat_pts, pts_to_row


start = time.time()
flat_pts, pts_to_row = get_data()
cert, grndpts = system.controller(flat_pts, pts_to_row,True)
end = time.time()
print(f"{end-start}s to get certificate")

start = time.time()
accepted = system.interlock(cert)
end = time.time()
print(f"{end-start}s for interlock response: {accepted}")

print(f'\n{len(grndpts)} plane pts, {len(cert.high_patch["pts"])} patch pts')
plt.double_plot([], grndpts, cert.high_patch['pts'])

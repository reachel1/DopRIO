import os
import csv
import argparse

def read_data(filename):
    col1, col2, col3, col4, col5, col6, col7 = [], [], [], [], [], [], []
    with open(filename, newline='') as csvfile:
        datareader = csv.reader(csvfile, delimiter=' ')
        for row in datareader:
            col1.append(float(row[1]))
            col2.append(float(row[2]))
            col3.append(float(row[3]))
            col4.append(float(row[4]))
            col5.append(float(row[5]))
            col6.append(float(row[6]))
            col7.append(float(row[7]))
    return col1, col2, col3, col4, col5, col6, col7

def write_lists_to_file(filename, *lists):
    with open(filename, 'w') as f:
        n = len(lists[0])  # Assume that each list is the same length
        for i in range(n):
            line = ''
            for lst in lists:
                line += str(lst[i]) + '\t'
            line = line[:-1]  # Remove the last TAB
            f.write(line + '\n')

def quaternion2RotateMatrix(q1, q2, q3, qw):
    R1, R2, R3, R4, R5, R6, R7, R8, R9 = [], [], [], [], [], [], [], [], []
    for i in range(len(q1)):
        r1 = 1 - 2*(q2[i]*q2[i] + q3[i]*q3[i])
        r2 = 2*(q1[i]*q2[i] - qw[i]*q3[i])
        r3 = 2*(q1[i]*q3[i] + qw[i]*q2[i])
        r4 = 2*(q1[i]*q2[i] + qw[i]*q3[i])
        r5 = 1 - 2*(q1[i]*q1[i] + q3[i]*q3[i])
        r6 = 2*(q2[i]*q3[i] - qw[i]*q1[i])
        r7 = 2*(q1[i]*q3[i] - qw[i]*q2[i])
        r8 = 2*(q2[i]*q3[i] + qw[i]*q1[i])
        r9 = 1 - 2*(q1[i]*q1[i] + q2[i]*q2[i])
        R1.append(r1)
        R2.append(r2)
        R3.append(r3)
        R4.append(r4)
        R5.append(r5)
        R6.append(r6)
        R7.append(r7)
        R8.append(r8)
        R9.append(r9)
    return R1, R2, R3, R4, R5, R6, R7, R8, R9

def tum_to_kitti(tum_file_path, kitti_file_path):
    tum_filename = tum_file_path # filename
    kitti_filename = kitti_file_path # filename
    if os.path.isfile(tum_filename): # Check whether the file exists
        t1, t2, t3, q1, q2, q3, qw = read_data(tum_filename)
        R1, R2, R3, R4, R5, R6, R7, R8, R9 = quaternion2RotateMatrix(q1, q2, q3, qw)
        write_lists_to_file(kitti_filename, R1, R2, R3, t1, R4, R5, R6, t2, R7, R8, R9, t3)
    else:
        print(f"{tum_filename} does not exist.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert TUM trajectory to KITTI format")
    parser.add_argument('tum_file_path', help="Input TUM file path")
    parser.add_argument('kitti_file_path', help="Output KITTI file path")
    args = parser.parse_args()
    
    tum_to_kitti(args.tum_file_path, args.kitti_file_path)

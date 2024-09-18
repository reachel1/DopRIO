import sys
import matplotlib.pyplot as plt

# use this .py could plot cov of rise
# python3 plot_cov_diag.py covfile.txt

def read_file(filename):
    data = []
    with open(filename, 'r') as file:
        for line in file:
            values = list(map(float, line.split()))
            if len(values) == 21:
                data.append(values)
            else:
                print(f"Warning: Line skipped due to incorrect number of values: {line}")
    return data

def plot_data(data):
    num_lines = len(data)
    num_values = len(data[0])
    
    x = list(range(num_lines))
    
    fig, axes = plt.subplots(7, 1, figsize=(10, 20))
    fig.tight_layout(pad=5.0)

    titles = ['p IinG', 'v inG', 'R ItoG', 'bias gyr', 'bias acc', 'R rtoi', 'p rini']
    
    for i in range(7):
        y1 = [data[j][i*3] for j in range(num_lines)]
        y2 = [data[j][i*3 + 1] for j in range(num_lines)]
        y3 = [data[j][i*3 + 2] for j in range(num_lines)]
        
        axes[i].plot(x, y1, label='x')
        axes[i].plot(x, y2, label='y')
        axes[i].plot(x, y3, label='z')
        
        axes[i].set_xlabel('Line Number')
        axes[i].set_ylabel('Value')
        axes[i].set_title(titles[i])
        axes[i].legend()
    
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python plot_lines.py <filename>")
        sys.exit(1)

    filename = sys.argv[1]
    data = read_file(filename)
    if data:
        plot_data(data)
    else:
        print("No data to plot.")
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook

import numpy as np
import pandas as pd

df_rrinverse = pd.read_csv('ReconROS/inverse.csv', delimiter=';')
df_rrsobel = pd.read_csv('ReconROS/sobel.csv', delimiter=';')
df_rrsort = pd.read_csv('ReconROS/sort.csv', delimiter=';')
df_rrmnist = pd.read_csv('ReconROS/mnist.csv', delimiter=';')
df_rrperiodic = pd.read_csv('ReconROS/periodic.csv', delimiter=';')

df_rrhwinverse = pd.read_csv('ReconROS_HW/inverse.csv', delimiter=';')
df_rrhwsobel = pd.read_csv('ReconROS_HW/sobel.csv', delimiter=';')
df_rrhwsort = pd.read_csv('ReconROS_HW/sort.csv', delimiter=';')
df_rrhwmnist = pd.read_csv('ReconROS_HW/mnist.csv', delimiter=';')
df_rrhwperiodic = pd.read_csv('ReconROS_HW/periodic.csv', delimiter=';')

df_inverse = pd.read_csv('Default/inverse.csv', delimiter=';')
df_sobel = pd.read_csv('Default/sobel.csv', delimiter=';')
df_sort = pd.read_csv('Default/sort.csv', delimiter=';')
df_mnist = pd.read_csv('Default/mnist.csv', delimiter=';')
df_periodic = pd.read_csv('Default/periodic.csv', delimiter=';')
#print(df_inverse)


#RR DATA
rrinverse_time = df_rrinverse['Inverse'][1001]
print(rrinverse_time)
rrinverse_data = df_rrinverse['Inverse'][1:1000]

rrmnist_time = df_rrmnist['Mnist'][1001]
print(rrmnist_time)
rrmnist_data = df_rrmnist['Mnist'][1:1000]

rrsobel_time = df_rrsobel['Sobel'][1001]
print(rrsobel_time)
rrsobel_data = df_rrsobel['Sobel'][1:1000]

rrsort_time = df_rrsort['Sort'][1000]
print(rrsort_time)
rrsort_data = df_rrsort['Sort'][1:1000]

rrperiodic_time = df_rrperiodic['Periodic'][1000]
print(rrperiodic_time)
rrperiodic_data = df_rrperiodic['Periodic'][1:1000]


#RR DATA
rrhwinverse_time = df_rrhwinverse['Inverse'][1001]
print(rrhwinverse_time)
rrhwinverse_data = df_rrhwinverse['Inverse'][1:1000]

rrhwmnist_time = df_rrhwmnist['Mnist'][1001]
print(rrhwmnist_time)
rrhwmnist_data = df_rrhwmnist['Mnist'][1:1000]

rrhwsobel_time = df_rrhwsobel['Sobel'][1001]
print(rrhwsobel_time)
rrhwsobel_data = df_rrhwsobel['Sobel'][1:1000]

rrhwsort_time = df_rrhwsort['Sort'][1000]
print(rrhwsort_time)
rrhwsort_data = df_rrhwsort['Sort'][1:1000]

rrhwperiodic_time = df_rrhwperiodic['Periodic'][1000]
print(rrhwperiodic_time)
rrhwperiodic_data = df_rrhwperiodic['Periodic'][1:1000]

#NONRR DATA
inverse_time = df_inverse['Inverse'][1001]
print(inverse_time)
inverse_data = df_inverse['Inverse'][1:1000]

mnist_time = df_mnist['Mnist'][1001]
print(mnist_time)
mnist_data = df_mnist['Mnist'][1:1000]

sobel_time = df_sobel['Sobel'][1001]
print(sobel_time)
sobel_data = df_sobel['Sobel'][1:1000]

sort_time = df_sort['Sort'][1000]
print(sort_time)
sort_data = df_sort['Sort'][1:1000]

periodic_time = df_periodic['Periodic'][1000]
print(periodic_time)
periodic_data = df_periodic['Periodic'][1:1000]

binmin = 0
binmax = 1000
binwidth = 10

xlimarr = [200,200,200,400,200,400]

bins = range(binmin, binmax + binwidth, binwidth)
print(bins)
fig, axs = plt.subplots(3, 6)

axs[0,0].text(0.5, 0.5,"Standard\n ROS 2\nExecutor"         ,fontsize=12,horizontalalignment='center',verticalalignment='center')
axs[1,0].text(0.5, 0.5,"ReconROS\n Executor\n HW and SW"    ,fontsize=12,horizontalalignment='center',verticalalignment='center')
axs[2,0].text(0.5, 0.5,"ReconROS\n Executor\n HW"           ,fontsize=12,horizontalalignment='center',verticalalignment='center')

axs[0, 1].hist(inverse_data, bins=bins, density=True, label='Avg {:.2f}'.format(np.average(inverse_data),np.var(inverse_data)))
axs[0, 1].vlines(x=np.average(inverse_data), ymin=0, ymax=100, colors='k', linestyles='dashed' )
axs[1, 1].hist(rrinverse_data, bins=bins,density=True,label='Avg {:.2f}'.format(np.average(rrinverse_data),np.var(rrinverse_data)))
axs[1, 1].vlines(x=np.average(rrinverse_data), ymin=0, ymax=100, colors='k', linestyles='dashed' )
axs[2, 1].hist(rrhwinverse_data, bins=bins,density=True,label='Avg {:.2f}'.format(np.average(rrhwinverse_data),np.var(rrhwinverse_data)))
axs[2, 1].vlines(x=np.average(rrhwinverse_data), ymin=0, ymax=100, colors='k', linestyles='dashed' )

axs[0, 2].hist(mnist_data, bins=bins,density=True,label='Avg {:.2f}'.format(np.average(mnist_data),np.var(mnist_data)))
axs[0, 2].vlines(x=np.average(mnist_data), ymin=0, ymax=100, colors='k', linestyles='dashed' )
axs[1, 2].hist(rrmnist_data, bins=bins,density=True,label=' Avg {:.2f}'.format(np.average(rrmnist_data),np.var(rrmnist_data)))
axs[1, 2].vlines(x=np.average(rrmnist_data), ymin=0, ymax=100, colors='k', linestyles='dashed' )
axs[2, 2].hist(rrhwmnist_data, bins=bins,density=True,label='Avg {:.2f}'.format(np.average(rrhwmnist_data),np.var(rrhwmnist_data)))
axs[2, 2].vlines(x=np.average(rrhwmnist_data), ymin=0, ymax=100, colors='k', linestyles='dashed' )

axs[0, 3].hist(sobel_data, bins=bins,density=True,label='Avg {:.2f}'.format(np.average(sobel_data),np.var(sobel_data)))
axs[0, 3].vlines(x=np.average(sobel_data), ymin=0, ymax=100, colors='k', linestyles='dashed' )
axs[1, 3].hist(rrsobel_data, bins=bins,density=True,label='Avg {:.2f}'.format(np.average(rrsobel_data),np.var(rrsobel_data)))
axs[1, 3].vlines(x=np.average(rrsobel_data), ymin=0, ymax=100, colors='k', linestyles='dashed' )
axs[2, 3].hist(rrhwsobel_data, bins=bins,density=True,label='Avg {:.2f}'.format(np.average(rrhwsobel_data),np.var(rrhwsobel_data)))
axs[2, 3].vlines(x=np.average(rrhwsobel_data), ymin=0, ymax=100, colors='k', linestyles='dashed' )

axs[0, 4].hist(sort_data, bins=bins,density=True,label='Avg {:.2f}'.format(np.average(sort_data),np.var(sort_data)))
axs[0, 4].vlines(x=np.average(sort_data), ymin=0, ymax=100, colors='k', linestyles='dashed' )
axs[1, 4].hist(rrsort_data, bins=bins,density=True,label='Avg {:.2f}'.format(np.average(rrsort_data),np.var(rrsort_data)))
axs[1, 4].vlines(x=np.average(rrsort_data), ymin=0, ymax=100, colors='k', linestyles='dashed' )
axs[2, 4].hist(rrhwsort_data, bins=bins,density=True,label='Avg {:.2f}'.format(np.average(rrhwsort_data),np.var(rrhwsort_data)))
axs[2, 4].vlines(x=np.average(rrhwsort_data), ymin=0, ymax=100, colors='k', linestyles='dashed' )

axs[0, 5].hist(periodic_data, bins=bins,density=True, label='Avg {:.2f}'.format(np.average(periodic_data),np.var(periodic_data)))
axs[0, 5].vlines(x=np.average(periodic_data), ymin=0, ymax=100, colors='k', linestyles='dashed' )
axs[1, 5].hist(rrperiodic_data, bins=bins,density=True, label='Avg {:.2f}'.format(np.average(rrperiodic_data),np.var(rrperiodic_data)))
axs[1, 5].vlines(x=np.average(rrperiodic_data), ymin=0, ymax=100, colors='k', linestyles='dashed' )
axs[2, 5].hist(rrhwperiodic_data, bins=bins,density=True, label='Avg {:.2f}'.format(np.average(rrhwperiodic_data),np.var(rrhwperiodic_data)))
axs[2, 5].vlines(x=np.average(rrhwperiodic_data), ymin=0, ymax=100, colors='k', linestyles='dashed' )

for i in range(0,3):
    axs[i, 0].axis('off')

    if i==0:
        axs[i, 1].set_title('Inverse')

    if i==0:
        axs[i, 2].set_title('Mnist')

    if i==0:
        axs[i, 3].set_title('Sobel')    

    if i==0:
        axs[i, 4].set_title('Sort')

    if i==0:
        axs[i, 5].set_title('Periodic')
    
    

    for j in range(1,6):
        #axs[i, j].set_xlabel('roundtrip [ms]', loc='right', labelpad=0, fontsize=10)
        axs[i, j].set_xticks([xlimarr[j]/2, xlimarr[j]])
        axs[i, j].set_xticklabels(['{} ms'.format(xlimarr[j]/2), '{} ms'.format(xlimarr[j])], fontsize=10)
        axs[i, j].set_xlim([0, xlimarr[j]])
        axs[i, j].set_ylim([0, 0.1])

        #axs[i, j].legend()

plt.show()
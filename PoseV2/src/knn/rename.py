import os
path = '/home/ox05/Documents/knn/data'
for file in os.listdir(path):
    # print('{}/{}.png'.format(path,file))
    os.rename('{}/{}'.format(path,file),'{}/{}.png'.format(path,file))

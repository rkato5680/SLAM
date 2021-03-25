import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def bresenham2D(sx, sy, ex, ey):
  '''
  Bresenham's ray tracing algorithm in 2D.
  Inputs:
	  (sx, sy)	start point of ray
	  (ex, ey)	end point of ray
  '''
  sx = int(round(sx))
  sy = int(round(sy))
  ex = int(round(ex))
  ey = int(round(ey))
  dx = abs(ex-sx)
  dy = abs(ey-sy)
  steep = abs(dy)>abs(dx)
  if steep:
    dx,dy = dy,dx # swap 

  if dy == 0:
    q = np.zeros((dx+1,1),dtype=int)
  else:
    q = np.append(0,np.greater_equal(np.diff(np.mod(np.arange( np.floor(dx/2), -dy*dx+np.floor(dx/2)-1,-dy),dx)),0))
  if steep:
    if sy <= ey:
      y = np.arange(sy,ey+1)
    else:
      y = np.arange(sy,ey-1,-1)
    if sx <= ex:
      x = sx + np.cumsum(q)
    else:
      x = sx - np.cumsum(q)
  else:
    if sx <= ex:
      x = np.arange(sx,ex+1)
    else:
      x = np.arange(sx,ex-1,-1)
    if sy <= ey:
      y = sy + np.cumsum(q)
    else:
      y = sy - np.cumsum(q)
  return np.vstack((x,y))


if  __name__ == "__main__":
	# loading csv files
	df_encoder = pd.read_csv("sensor_data/encoder.csv", header=None)
	df_encoder.columns =  ["timestamp", "left count", "right count"]

	df_fog = pd.read_csv("sensor_data/fog.csv", header=None)
	df_fog.columns =  ["timestamp", "delta roll", "delta pitch", "delta yaw"]

	df_lidar = pd.read_csv("sensor_data/lidar.csv", header=None)


	# loading R matrix and T vector for tranforming lidar frame to the world frame
	R = list(map(float, "0.00130201 0.796097 0.605167 0.999999 -0.000419027 -0.00160026 -0.00102038 0.605169 -0.796097".split()))
	R = np.array(R).reshape(3,3)

	T = list(map(float, "0.8349 -0.0126869 1.76416".split()))
	T = np.array(T)


	# creating dataframe combinining all necessary information for each time step

	start = 1544582648800000000
	end =   1544583809200000000
	#stepsize
	step = 1000000000

	diam_left = 0.623479
	diam_right = 0.622806

	df_tmp = df_encoder[np.logical_and((start<=df_encoder.iloc[:, 0]), (df_encoder.iloc[:, 0]<=end))]
	df_tmp.iloc[:, 1] *= diam_left*np.pi/4096
	df_tmp.iloc[:, 2] *= diam_right*np.pi/4096

	df2 = pd.DataFrame(df_tmp.iloc[1:, 0])
	df2["timedelta"] = df_tmp.iloc[:, 0].diff()[1:] / 10**9 #ここで秒に戻す
	df2["delta_x"] = df_tmp.iloc[:, 1:].mean(1).diff()
	df2 = df2.reset_index(drop=True)

	df_tmp0 = df_fog[np.logical_and((start<=df_fog.iloc[:, 0]), (df_fog.iloc[:, 0]<=end))]
	df_tmp = pd.DataFrame(df_tmp0.iloc[:, 0])
	df_tmp["yaw"] = df_tmp0.iloc[:, 3].cumsum()
	df2["delta_yaw"] = df_tmp["yaw"][::10].diff().iloc[1:len(df2)+1].to_numpy()

	df2["v"] = df2["delta_x"] / df2["timedelta"]
	df2["omega"] = df2["delta_yaw"] / df2["timedelta"]


	# computing initial value of mean and covariance for prediciton of each particel
	# equivalent to sample mean and covariance
	df_tmp = df2[np.logical_and((start<=df2.iloc[:, 0]), (df2.iloc[:, 0]<=start+step))]
	mean = df_tmp.iloc[:, 4:].mean(0)
	cov = df_tmp.iloc[:, 4:].cov()


	###################################################################################
	# main part of SLAM 
	###################################################################################
	# parameters
	n_particle = 100
	gridmap = np.zeros((1500,1400))
	rmin,rmax =2,75 #cutoff of Lidar data
	log4 = np.log(4)
	lmin,lmax = -5,5 #min and max of cumulative log-odds
	ldecay = 0.999 # decay of log-odds
	dx,dy = 100,1200 # adjustment to grid map coordinate
	scale_pa_var = np.array([[1,1],[1,1]]) # scaling factor of covariance

	# resampling parameters
	thr_v_resampling = 1/n_particle / 2
	thr_n_resampling = n_particle // 5

	gridmap_list = []

	n_step = (end - start + step - 1)//step

	# pos[i, j, k]: estimated position (x,y,theta) for i-th time step and j-th particle
	pos_pa = np.zeros((n_step, n_particle, 3))
	weights = np.zeros((n_step, n_particle))
	weights[0] = np.ones(n_particle) / n_particle

	# estimated position by summing esitimation of paricles
	pos = np.zeros((n_step, 3))

	# observed position
	pos_ob = np.zeros((n_step, 3))

	tau = step / 10**9
	for i, s in enumerate(range(start+step, end, step)):
	    # prediction of each particle
	    v,omega = np.random.multivariate_normal(mean, cov * scale_pa_var, n_particle).T
	    
	    theta = pos_pa[i, :, 2] + omega * tau
	    x = pos_pa[i, :, 0] + v*np.cos(theta) * tau
	    y = pos_pa[i, :, 1] + v*np.sin(theta) *tau
	    pos_pa[i+1, :, :] = np.vstack([x,y,theta]).T
	    
	    # search for nearlest Lidar data in term of timestamp using binary search
	    middle = s + step//2
	    idx = np.searchsorted(df_lidar.iloc[:, 0], middle)
	    tmp = df_lidar.iloc[idx, 1:]
	    tmp = tmp[np.logical_and((rmin<=tmp), (tmp<=rmax))]
	    indices = tmp.index
	    args = (indices*0.666 + -5)/180*np.pi

	    m = np.vstack([tmp*np.cos(args), tmp*np.sin(args)]).T
	    
	    # coordinate transformation
	    tmp = np.zeros((m.shape[0], 3))
	    tmp[:, :2] = m
	    m = (R @ (tmp+T).T).T[:, :2]
	    
	    # Computing weight of particles using map correlation
	    if i == 0: weights[i+1, :] = weights[i, :].copy()
	    else:
	        corrs = np.zeros(n_particle, int)
	        for pa_idx, (x,y,theta) in enumerate(pos_pa[i+1]):
	            rot_mat = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
	            m_pa = (rot_mat @ m.T).T
	            m_pa[:, 0] += x
	            m_pa[:, 1] += y
	            x0,y0 = int(x)+dx,int(y)+dy
	            m_pa = m_pa.astype(int)
	            m_pa[:, 0] += dx
	            m_pa[:, 1] += dy

	            # To avoid complexity, only compare correlation of negative values
	            # To avoid complexity, only compare 200 x 200 sized small grid
	            grid_tmp = np.zeros((200,200), int)
	            for x,y in m_tmp:
	                xs,ys = bresenham2D(100,100,x-(x0-100),y-(y0-100))
	                grid_tmp[xs,ys] = -1

	            # computing map correlation
	            corrs[pa_idx] = np.count_nonzero(np.logical_and((gridmap[x0-100:x0+100,y0-100:y0+100]<0), (grid_tmp<0)))

	        weights[i+1, :] = corrs / corrs.sum()
	    
	    # computing single estimated position by summing predictions of particles
	    pos[i+1,:] = (pos_pa[i+1, :, :] * weights[i+1, :, None]).sum(0)
	    
	    # transform Lidar data to the world frame
	    theta = pos[i+1,2]
	    rot_mat = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
	    m = (rot_mat @ m.T).T
	    
	    m[:, 0] += pos[i+1, 0]
	    m[:, 1] += pos[i+1, 1]
	    x0,y0 = int(pos[i+1, 0])+dx,int(pos[i+1, 1])+dy
	    m = m.astype(int)
	    m[:, 0] += dx
	    m[:, 1] += dy
	    
	    m_tmp = sorted(m, key=lambda x: (abs(x[0]-x0), abs(x[1]-y0)), reverse=True)
	    
	    # To avoid overwrapping, aggreage unoccupied and occupied cells in temporary grid and then add it to the original grid map.
	    grid_tmp = np.zeros(gridmap.shape)
	    for x,y in m_tmp:
	        xs,ys = bresenham2D(x0,y0,x,y)
	        grid_tmp[xs,ys] = -log4
	        grid_tmp[x,y] = log4
	        
	    gridmap += grid_tmp
	    
	    # maintaining log-odds in small range
	    gridmap[gridmap < lmin] = lmin
	    gridmap[gridmap > lmax] = lmax
	    
	    if i and i % 100 == 0:
	        print(f"{i} -th iteration done...")
	        gridmap_list.append(gridmap.copy())
	        
	    gridmap *= ldecay
	    
	    # Computing sample mean and covariance of linear velocity and angular velocity for prediction in the next time step
	    df_tmp = df2[np.logical_and((s<=df2.iloc[:, 0]), (df2.iloc[:, 0]<=s+step))]
	    mean = df_tmp.iloc[:, 4:].mean(0)
	    cov = df_tmp.iloc[:, 4:].cov()
	    
	    # Computing position using observed data
	    # it is not used for prediction
	    v_ob = df_tmp["v"].mean()
	    omega_ob = df_tmp["omega"].mean()
	    
	    theta_ob = pos_ob[i,2] + omega_ob * tau
	    x_ob = pos_ob[i,0] + v_ob*np.cos(theta_ob) * tau
	    y_ob = pos_ob[i,1] + v_ob*np.sin(theta_ob) *tau
	    pos_ob[i+1, :] = [x_ob, y_ob, theta_ob]
	    
	    # resampling
	    n_bellow = np.count_nonzero(weights[i+1,:] < thr_v_resampling)
	    if n_bellow >= thr_n_resampling:
	        indices = np.random.choice(n_particle, n_particle, p=weights[i +1, :])
	        pos_pa[i+1,:,:] = pos_pa[i+1,indices,:]
	        weights[i+1,:] = weights[i+1, indices]
	        weights[i+1] /= weights[i+1].sum()

	############################################################################################



	    # plotting

	for i, gm in enumerate(gridmap_list):
	    if i == 0 or i % 10: continue
	    idx = (i+1)*100
	    vmin,vmax = 0,2
	    plt.figure(figsize=(20,20))
	    grid_tmp = 1-(gm<0)
	    grid_tmp[grid_tmp==vmin] = vmax
	    plt.imshow(grid_tmp.T, cmap='gray', vmin=vmin, vmax=vmax,origin="lower")

	    plt.plot(pos[:idx, 0]+dx, pos[:idx, 1]+dy, color="green",label="trajectory", linewidth=3)

	    plt.scatter(pos_pa[:idx, :10, 0].flatten()+dx, pos_pa[:idx, :10, 1].flatten()+dy,color="red", s=1, label="selected particles")
	    
	    plt.scatter(0,0,color="grey", label=f"T = {int((start + (idx + 1) * step)/10**9)}", s=0.001)

	    plt.axis("off")

	    plt.legend(prop={'size': 30}, markerscale=8)
	    plt.show()
	    
	idx = (i+1)*100
	vmin,vmax = 0,2
	plt.figure(figsize=(20,20))
	grid_tmp = 1-(gridmap<0)
	grid_tmp[grid_tmp==vmin] = vmax
	plt.imshow(grid_tmp.T, cmap='gray', vmin=vmin, vmax=vmax,origin="lower")

	plt.plot(pos[:idx, 0]+dx, pos[:idx, 1]+dy, color="green",label="trajectory", linewidth=3)

	plt.scatter(pos_pa[:idx, :10, 0].flatten()+dx, pos_pa[:idx, :10, 1].flatten()+dy,color="red", s=1, label="selected particles")

	plt.scatter(0,0,color="grey", label=f"T = {int((start + (idx + 1) * step)/10**9)}", s=0.001)

	plt.axis("off")

	plt.legend(prop={'size': 30}, markerscale=8)
	plt.show()



	# saving results

	#count_save = 0

	import pickle

	result = {"parameters": [n_particle, thr_v_resampling, thr_n_resampling, scale_pa_var],
	    "grid": gridmap, 
	          "grid_list": gridmap_list,
	          "estimated_position": pos, 
	          "positions of particles": pos_pa, 
	          "weight": weights}

	with open(f'result_{count_save}.pickle', 'wb') as handle:
	    pickle.dump(result, handle, protocol=pickle.HIGHEST_PROTOCOL)
	    
	count_save += 1

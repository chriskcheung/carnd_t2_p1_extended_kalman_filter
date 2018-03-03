
# carnd_t2_p1_extended_kalman_filter

[//]: # (Image References)
[image1]: ./../output/L_dataset1_result.png
[image2]: ./../output/L_dataset2_result.png
[image3]: ./../output/R_dataset1_result.png
[image4]: ./../output/R_dataset2_result.png
[image5]: ./../output/RL_dataset1_result.png
[image6]: ./../output/RL_dataset2_result.png


## Carnd - term 2 - project 1 - extended kalman filter

### Overview
The goal of this project is to build a extended Kalman Filter (EKF) model to process a series of sensor data provided by radar and lidar. 
The EKF model, which processes all incoming sensor data, is conected to a simulator, which generates both radar and lidar data, using uWebSocketIO.


### Project Repository
All resource are located in Udacity's project repository
[CarND-Extended-Kalman-Filter-Project](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)


### Project Submission
All modified code including results are committed to my personal github page
[carnd-t2-p1-extended-kalman-filter](https://github.com/chriskcheung/carnd_t2_p1_extended_kalman_filter)


### Key Files
##### main.cpp
establishes communication between simulator and EKF model using uWebSocketIO, and reads in data during set time interval and send sensor measurements to FusionEKF.cpp for processing 

##### FusionEKF.cpp
processes sensor data base on their stypes: radar vs lidar. FusionEKF initializes the tracked state x and prediction matrix P along with H merix, Jacobian, nosie R of equipment, Q etc. FusionEKF processes prediction of the current state x base on previous history. It also updates the current state x base on the measurement receive from radar and lidar. 

##### kalman_filter.cpp
base functions of Kalman filter, include predict(), update() for lidar, and updateEKF() for radar. 

##### tools.cpp
includes CalculateJacobian() for calculating Taylor series of Jacobian matrix and RMSE() for calculating the root means square ofthe predicted result veruse real groundtruth measurement
			


## Implementation Challenge

### Initialization
The first initialization needs to consider the different size of matrices for H and R. For first time initialization. if radar measurement input if received, use H_lasear and R_lasar, otherwise, if lidar measurement input is received, use Hj and R_radar. The size of these matrices are different: H_lasear(2,4), R_lasar(2,2), Hj(4,4) and R_radar(3,3). To use the kalman_filter::Init(), make sure the correct matrix size is declared before passing in the arguements including F, P, and Q. For radar initialization, px and py of state x needs to be converted from rho_measured, phi_measured, and rhodot_measured using the following.

```c++
      float px     = cos(phi)*rho;     // calculate position of x from radian metric phi and rho
	  float py     = sin(phi)*rho;     // calculate position of y from radian metric phi and rho
	  float vx     = cos(phi)*rhodot;  // calculate velocity of x from radian metric phi and rhodot
	  float vy     = sin(phi)*rhodot;  // calculate velocity of y from radian metric phi and rhodot
```	

The last, update the previous_timestamp with the timestamp from measurement input as all measured sensor data are processed at this time. 


### Prediction
After using the very first sensor data received for initialization, the sequential data will be used for prediction and state update. I will talk about update() in the next section. predict() is the same for both lidar and radar and can be shared. Using the delta time of previous_timestamp and the current measured timestamp and the noise requriement defined for x-axis and y-axis, the F and Q matrices will be updated accordingly.

```c++
   // update F matrix
   ekf_.F_ << 1, 0, dt, 0, 
              0, 1, 0, dt, 
			  0, 0, 1, 0, 
			  0, 0, 0, 1;

   // update Q matrix using noise_ax = 9 and noise_ay = 9
   float nax = 9.0; // noise_ax
   float nay = 9.0; // noise_ay
   float dt2 = dt*dt;
   float dt3 = dt2*dt;
   float dt4 = dt3*dt;
   
   ekf_.Q_ << dt4/4*nax, 0,         dt3/2*nax, 0,
              0,         dt4/4*nay, 0,         dt3/2*nay,
			  dt3/2*nax, 0,         dt2*nax,   0,
              0,         dt3/2*nay, 0,         dt2*nay;
```


### Update
Update for radar and lidar are different. Kalman filter update() for Lidar is quite straight forward by using state x directly. 

```c++
  // Lidar
  MatrixXd y  = z - H_*x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S  = H_*P_*Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K  = P_*Ht*Si;
  
  // new state
  x_ = x_ + K*y;
  P_ = (I - K*H_)*P_;
```

For Kalman filter for radar update, error equation y=z-H`*x` is replaced with y=z-h(x), and matrix H in the subsequent Kalman filter equations is replaced with Jacobian Hj. Notice that z is in radian space (rho, phi, rhodot), while x is in cartesian space. It is necessary to convert h(x) in cartesian space to h(rho, phi, rhodot) in polar coordinate to ensure the error equation are calculated in the same space. During the conversion, there is a chance for px and squareroot of px`*`px+py`*`py to be close to or equal zero which will caused phi and rhodot to be divided by zero. Sanity check should put in place to make sure divide by zero never happen. In my code, when it happens, I replace it with 0.1. Same goes to Jacobian calculation. If Hj is divided by zero, then skip the entire update of radar to avoid incorrect update to state x. 

```c++
  // Radar - converting h(x) from cartesian space to polar coordinate
  float d1 = sqrt(px*px + py*py);
  // Skip update if Hj is not valid
  float d2 = atan2(py,px);
  float d3 = d1 >= 0.00001 ? (px*vx + py*vy)/d1 : 0.1;

  h << d1, d2, d3;
```

```c++
  MatrixXd y  = z - h;
  MatrixXd Ht = Hj_.transpose();
  MatrixXd S  = Hj_*P_*Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K  = P_*Ht*Si;
  
  // new state
  x_ = x_ + K*y;
  P_ = (I - K*Hj_)*P_;
```


### Normalize phi Angle
It is crucial to normalize phi angle in the range between pi and -pi. To understand this, think of yourself driving a car, the range of turning can only goes from 90 degree from the center (straight) to the left(-) and to the right(+). In most cases, it is not event possible. When the wheel turns more then 90 to either side, the wheels of car will actually be pointing to the oppose direction in the forward motion (left becomes right, right becomes left). A more intuitive explanation can be found [Here](https://discussions.udacity.com/t/ekf-gets-off-track/276122/19?u=drivewell)

```c++
  // normalize phi, y(1) in this case to be within -pi to pi
  const double pi = 3.14159265358979323846;
  while(y(1) > pi){
    y(1) -= 2.*pi;
	cout << "kalman_filter(107) - y(1) > pi -> normalized to y(1)-2pi=" << y(1) << endl;
  }
  while(y(1) < -pi){
    y(1) += 2.*pi;
	cout << "kalman_filter(111) - y(1) < pi -> normalized to y(1)+2pi=" << y(1) << endl;
  }
```



## Result
At the beginning, I got segmentation fault for various mistaken, like passing in incorrect size of matrix to functions, not initializing matrix P upfront before using it in predict(). After these error was resolved, the new state of px, py, vx, vy were way off as shown in simulator. That inaccurate state was contributed by using incorrect x for CalculateJacobian(x) and h(x), i.e. derived a set of px,py,vx,vy instead of using the predicted state x. After revisting the description from lecture, I realized that predicted state x should be used and from there the result from simulator start showing the corrected tracked state X. However, it only worked for the positive x and y region. Further debug found that it was contributed by not normailizing the phi angel. As a result, normalization was added and result became better. RMSE started with above the rubric requirement, but the number dropped as the simulator went on and eventually hit below the 0.1 and 0.52 for x y positions and vx vy velocity. I also tried plotting the state x for Lidar only, Radar only and combined Radar and Lidar sensor measurement to observe their difference. Here are what I observed. 

#### Ladar only measurements
Lidar only measurement did show good result in terms of tracking the kalman filter states on dateset1 while it slightly went off on dataset2. The RMSE stayed above the required rubric guideline.
##### dataset 1
![alt text][image1]

##### dataset 2
![alt text][image2]

#### Radar only measurements
Ridar only measurement showed that it went off tracked during the curve turning for both dataset 1 and 2. The RMSE for both dataset went above the accruacy requirement by double.
##### dataset 1
![alt text][image3]

##### dataset 2
![alt text][image4]

#### Combined Radar and Ladar measurements
Combined Radar and Lidar measurement showed good result in terms of tracking the kalman filter states and RMSE.
##### dataset 1
![alt text][image5]

##### dataset 2
![alt text][image6]

Overall, the project is not diffcult, it just require to follow the equations provided and use the correct data and right matrix size at each step. 


'''
***************************************************************************
Modified Inverse Kinematics solver for UR5 from Modern Robotics Course
***************************************************************************
Author: Alexandros Mantzios
Credits: Modern Robotics: Mechanics, Planning, and Control. Code Library
Email: alexandrosmantzios@gmail.com
Date: January 2022
***************************************************************************
Language: Python
Required library: numpy, modern_robotics core library
Optional library: matplotlib
***************************************************************************
'''

'''
*** IMPORTS ***
'''


import numpy as np
from modern_robotics import JacobianBody, MatrixLog6, TransInv, FKinBody, se3ToVec
import math as math
from pathlib import Path
import zipfile

PROJECT_DIR = Path(__file__).parent
   

class UR5robot:
    """Class of UR5 Robot """
    def __init__(self, 
                 W1=0.109,
                 W2=0.089, 
                 L1=0.425, 
                 L2=0.392, 
                 H1=0.089, 
                 H2=0.095) -> None:
        self.W1 = W1
        self.W2 = W2
        self.L1 = L1
        self.L2 = L2
        self.H1 = H1
        self.H2 = H2 

    
    @property
    def home_position(self) -> np.ndarray:
        "End effector frame {b} in the zero position"
        return np.array([
                  [-1, 0, 0, self.L1+self.L2],
                  [ 0, 0, 1, self.W1+self.W2],
                  [ 0, 1, 0, self.H1-self.H2],
                  [ 0, 0, 0, 1]])
        

    @property
    def screw_axes_body(self) -> np.ndarray:
        "returns 6X6 matrix of screw axis with respect to body frame (robot flange)"
        
        omg1 = [0,1,0]
        omg2 = [0,0,1]
        omg3 = [0,0,1]
        omg4 = [0,0,1]
        omg5 = [0,-1,0]
        omg6 = [0,0,1]

        v1 = [self.W1+self.W2, 0, self.L1+self.L2]
        v2 = [self.H2, -self.L1-self.L2, 0]
        v3 = [self.H2, -self.L2, 0]
        v4 = [self.H2, 0, 0]
        v5 = [-self.W2, 0, 0]
        v6 = [0,0,0]   

        b1= omg1 + v1
        b2= omg2 + v2
        b3= omg3 + v3
        b4= omg4 + v4
        b5= omg5 + v5
        b6= omg6 + v6

        return np.array([b1,b2,b3,b4,b5,b6]).T  
    
    @property
    def desired_pose(self):
        return np.array([
                  [0, 1, 0, -0.5],
                  [0, 0, -1, 0.1],
                  [-1, 0, 0, 0.1],
                  [0, 0, 0, 1]
                  ])
        
class IK_NoSolution(Exception):
    """the inverse kinematics did not converge"""
    pass

def IKinBodyIterates(screw_axes_body:np.ndarray, 
                     home_position:np.ndarray,
                     desired_pose:np.ndarray,
                     thetalist0:np.ndarray,
                     angular_error,
                     linear_error):
    
    import zipfile
    # Specification of output files
    CSV_PATH='/Users/alexa/OneDrive/Υπολογιστής/Modern Robotics/Alex_Test/iterates.csv'
    LOG_FILE='/Users/alexa/OneDrive/Υπολογιστής/Modern Robotics/Alex_Test/log.txt'
    # Delete output files if they exist
    if LOG_FILE.exists():
        LOG_FILE.unlink()
    if CSV_PATH.exists():
        CSV_PATH.unlink()
    
    #Starting guess
    thetalist = np.array(thetalist0).copy()

    # joint vectors 
    thetalist_1=np.array([thetalist])
    np.set_printoptions(precision=2)

    # iteration counter
    i = 0

    # Definition of max iterations of the Newton Raphson
    maxiterations = 20

    # print iteration number
    print(f"Iteration number{i}=")

    # print joint vector list
    print(f"joint vector at iteration={thetalist}")

    # to compute error we need to convert the SE(3) matrix of ee configuration to
    Vb \
        = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(home_position, 
                                                       screw_axes_body, \
                                                       thetalist)), desired_pose)))
    np.set_printoptions(precision=7)
    print("Original Twist Vb:" , Vb)

    #calculate error 
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > angular_error \
          or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > linear_error
    
    # iterate for error
    while err and i < maxiterations:
         # print iteration number
        print(f"Iteration number{i}=")
        # next joint vector
        thetalist = thetalist \
                    + np.dot(np.linalg.pinv(JacobianBody(screw_axes_body, \
                                                         thetalist)), Vb)

       ### new calculated joint vector. put in matrix with all iterations 
        thetalist_1=np.vstack([thetalist_1, thetalist])
        #find end effector configuration SE(3)
        ee_conf = FKinBody(home_position, screw_axes_body,thetalist)
        np.set_printoptions(precision=2)
        print(f"SE(3) end- effector config: {ee_conf}")
       # print joint vector list
        np.set_printoptions(precision=7)
        print(f"joint vector at iteration={thetalist}")
        
        # calculate new twist 
        Vb = se3ToVec(MatrixLog6(np.dot(TransInv(FKinBody(home_position,
                                                           screw_axes_body, \
                                                       thetalist)), desired_pose)))
        np.set_printoptions(precision=7)
        print(f"error twist V_b: {Vb}")

        # calculate new angular error
        angular_error_new=np.linalg.norm([Vb[0], Vb[1], Vb[2]])
        print(f"angular error magnitude: ||omega_b||: {angular_error_new}")

        # calculate new linear error 
        linear_error_new= np.linalg.norm([Vb[3], Vb[4], Vb[5]])
        print(f"linear error magnitude ∣∣v_b∣∣ : {linear_error_new}")
        #print(thetalist_1)

        # Calculate error
        err = angular_error_new > angular_error \
              or linear_error_new > linear_error
        
        #counter
        i = i + 1
        
    # # Alternative way to save the matrix to csv file (Uncomment if necessary)
    # np.savetxt("iterates.csv", thetalist_1, delimiter=",")
    # # return thetalist and whether solution found (True for found, False for not)   
    CSV_PATH='/Users/alexa/OneDrive/Υπολογιστής/Modern Robotics/Alex_Test/iterates.csv'
    LOG_FILE='/Users/alexa/OneDrive/Υπολογιστής/Modern Robotics/Alex_Test/log.txt'
    with open(LOG_FILE, "a", encoding="utf-8") as log: 
        log.write(f"Iteration,{i}\n")
        log.write(f" joint vector : {thetalist.round(3)}\n")
        log.write(f" SE(3) end-effector config : {ee_conf}\n")
        log.write(f" error twist V_b: {Vb}")
        log.write(f" angular error magnitude ∣∣omega_b∣∣: {angular_error}\n")
        log.write(f" linear error magnitude ||v_b||: {linear_error}\n")

    with open(CSV_PATH, "w", encoding="utf-8") as csv_file:
        for thetalist_iter in thetalist:
            csv_file.write(str(thetalist_iter) + "\n")
            # csv_file.write(",".join([str(c) for c in thetalist_iter])+"\n")

    "adds results into a zip file"
    # Replace with appropriate path
    PROJECT_DIR = pathlib.Path('/Users/alexa/OneDrive/Υπολογιστής/Modern Robotics/Alex_Test')
    zip_file_path = PROJECT_DIR / "Alex.zip"
    zip_file_path.unlink(missing_ok=True)
    files_to_include = [
        PROJECT_DIR / "iterates.csv",
        PROJECT_DIR / "log.txt",
        PROJECT_DIR / "IKinBodyIterates.py",
        PROJECT_DIR / "screenshots.png",
        PROJECT_DIR / "video.avi"
    ]

    with zipfile.ZipFile(zip_file_path, "w") as zf:
        for file in files_to_include:
            if file.is_file():
                zf.write(file,arcname=file.relative_to(PROJECT_DIR))
            else:
                print(f"File {file.name} does not exist")

    return (thetalist, not err)



if __name__ == "__main__":

 ur5=UR5robot()

# initial guess of joint angles - Needs to be close to initial T matrix 
theta_guess= np.array([-0.421,-2.160,4.685,1.204,3.157,1.765])
angular_error1 = 0.001
linear_error1 = 0.0001
thetalist_final, success =IKinBodyIterates(ur5.screw_axes_body, ur5.home_position, ur5.desired_pose, theta_guess,angular_error1,linear_error1)
if not success or abs(thetalist_final[0]) > 6 or abs(thetalist_final[1]) > 6 or abs(thetalist_final[2]) > 6 or abs(thetalist_final[3]) > 6 or abs(thetalist_final[4]) > 6 or abs(thetalist_final[5]) > 6 :  
    raise IK_NoSolution("No IK solution found")
else:
    print("Solution joint vectors" ,thetalist_final, success)







    


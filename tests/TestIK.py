

# Test the IK on a set of predefined joint poses and plot accordingly using ISB_UL arm model
def _testIK(debug=False):

    postures_to_test=[[0, 0, 0, 0], #Arm along body; ISB angles singularity
                      [90, 0, 0, 90], #Elbow flex 90 degrees forward
                      [90, 20, 0, 90], #Elbow flex 90 degrees forward, slight elevation
                      [90, 20, -20, 90], #Elbow flex 90 degrees som int rotation
                      [0, 20, 0, 90], #Elbow flex, full external
                      [45, 45, -45, 90], #Elbow flex, mid elevation, mid ext. with int rotation
                      [120, 30, 45, 30], #Elbow ext, full internal, mid ele with some ext rot
                      [120, 30, 25, 30] #
                      ]

    #Define ISB rtb arm model
    arm_model_params_d = {'ua_l': 0.3,
                          'fa_l': 0.25,
                          'ha_l': 0.1,
                          'm_ua': 2.0,
                          'm_fa': 1.1+0.23+0.6}
    ISB_UL = Deweighting(arm_model_params_d)

    #Define test set:
    to_test=[]
    for posture in postures_to_test:

        #Compute theoretical link positions for each pos
        jointsPos={}
        XX=ISB_UL.ISB_UL.fkine_all(np.deg2rad(np.array(posture+[0,0,0]))).t
        jointsPos[J.R_S]=np.array(XX[1])*1000 #expected in mm by IK
        jointsPos[J.R_E]=np.array(XX[3])*1000
        jointsPos[J.R_W]=np.array(XX[5])*1000
        #Add required trunk/face/shoulder joints pos
        jointsPos[J.L_S]=np.array(XX[1])+[-200, 0, 0]
        jointsPos[J.L_Y]=np.array(XX[1])+[-120, 0, 10]
        jointsPos[J.R_Y]=np.array(XX[1])+[-80, 0, 10]

        to_test=to_test+[{'jointsPos': jointsPos, 'armSide': 'r', 'expected_q': posture}]

    #Apply and test IK on test set
    for test in to_test:
        q=IK(test['jointsPos'], test['armSide'])
        if(debug):
            ISB_UL.ISB_UL.plot(np.deg2rad(np.array(test['expected_q']+[0,0,0])), backend='pyplot', block=False)
            print('At joints pos:', test['jointsPos'], '\n=>', np.rad2deg(np.array(q)).round(), ' =?= ', np.array(test['expected_q']).round(), '\n')
            fig2 = plt.figure()
            ISB_UL.ISB_UL.plot(q+[0,0,0], fig=fig2, backend='pyplot', block=True)
            plt.close('all')
        else:
            print('At joints pos:', test['jointsPos'], '\n=>', np.rad2deg(np.array(q)).round(), ' =?= ', np.array(test['expected_q']).round(), '\n')
            #np.testing.assert_allclose(np.rad2deg(np.array(q)).round(), (np.array(test['expected_q']).round()))

##3-DOF RPP Cylindrical Manipulator (Group 16_MEXE3203)

#MAY KASAMANG JACOBIAN INEZZZ

# THIS IS TO REMOVE THE PANDAS' FUTURE WARNING POPPING UP ON THE BOX
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

import PySimpleGUI as sg
import pandas as pd
import numpy as np
import math

# GUI code

sg.theme('DarkRed2')

# Excel read code

EXCEL_FILE = 'CYLINDRICAL.xlsx'
df = pd.read_excel(EXCEL_FILE)

# Lay-out code

layout = [
    [sg.Push(), sg.Text('Cylindrical RPP MEXE Calculator', font = ("Monotype Corsiva", 15)), sg.Push()],

    [sg.Text('Forward Kinematics Calculator', font = ("Cascadia Code", 12))],
    [sg.Text('Fill out the following fields:', font=("Cascadia Code", 10)),
     sg.Push(), sg.Button('Click this before Solving Forward Kinematics', font=("Cascadia Code", 15), size=(35, 0), button_color=('white', 'purple')),
     sg.Push()],

    [sg.Text('a1 = ', font = ("Cascadia Code" , 10)),sg.InputText('55', key='a1', size=(20,10)),
     sg.Text('T1 = ', font = ("Cascadia Code", 10)),sg.InputText('0', key= 'T1', size=(20,10)),
     sg.Push(), sg.Button('Jacobian Matrix (J)', font = ("Cascadia Code", 12), size=(15,0), button_color=('white', 'green')),
     sg.Button('Det(J)', font = ("Cascadia Code", 12), size=(15,0), button_color=('white', 'green')),
     sg.Button('Inverse of J', font = ("Cascadia Code", 12), size=(15,0), button_color=('white', 'green')),
     sg.Button('Transpose of J', font = ("Cascadia Code", 12), size=(15,0), button_color=('white', 'green')), sg.Push()],

    [sg.Text('a2 = ', font = ('Cascadia Code', 10)),sg.InputText('60', key='a2', size=(20,10)),
     sg.Text('d2 = ', font = ('Cascadia Code', 10)),sg.InputText('0', key='d2', size=(20,10))],

    [sg.Text('a3 = ', font=('Cascadia Code', 10)), sg.InputText('30', key='a3', size=(20, 10)),
     sg.Text('d3 = ', font=('Cascadia Code', 10)), sg.InputText('0', key='d3', size=(20, 10)),
     sg.Push(), sg.Button('Inverse Kinematics', font = ('Cascadia Code', 12), size=(35,0), button_color=('black', 'pink')), sg.Push()],

    [sg.Button('Solve Forward Kinematics', tooltip= 'Go first to "Click this before Solving Forward Kinematics"!', font = ('Cascadia Code', 12), button_color=('white', 'orange')), sg.Push(),
     sg.Push(),sg.Button('Path and Trajectory Planning', font = ("Cascadia Code", 12), size=(40,0), button_color=('white', 'blue')), sg.Push()],

    [sg.Frame('Position Vector: ',[[
        sg.Text('X = ', font = ("Cascadia Code", 10)), sg.InputText(key='X', size=(10,1)),
        sg.Text('Y = ', font = ("Cascadia Code", 10)), sg.InputText(key='Y', size=(10,1)),
        sg.Text('Z = ', font = ("Cascadia Code", 10)), sg.InputText(key='Z', size=(10,1))]])],

    [sg.Push(), sg.Frame('HO_3 Transformation Matrix = ', [[sg.Output(size=(60, 12))]]),
     sg.Push(), sg.Image('CYLINDRICAL.png'), sg.Push()],

    [sg.Submit(font=("Cascadia Code", 10)), sg.Exit(font=("Cascadia Code", 10))]]

# Window Code
window = sg.Window('CYLINDRICAL MEXE CALCULATOR', layout, resizable=True)

# Variable codes for disabling buttons

disable_J = window['Jacobian Matrix (J)']
disable_DetJ = window['Det(J)']
disable_IV = window['Inverse of J']
disable_TJ = window['Transpose of J']
disable_IK = window['Inverse Kinematics']
disable_PT = window['Path and Trajectory Planning']

while True:
    event, values = window.read()
    if event == sg.WIN_CLOSED or event == 'Exit':
        break

    if event == 'Click this before Solving Forward Kinematics':
        disable_J.update(disabled=True)
        disable_DetJ.update(disabled=True)
        disable_IV.update(disabled=True)
        disable_TJ.update(disabled=True)
        disable_IK.update(disabled=True)
        disable_PT.update(disabled=True)

    if event == 'Solve Forward Kinematics':
        # Forward Kinematics Codes

        # link lengths in cm
        a1 = float(values['a1'])
        a2 = float(values['a2'])
        a3 = float(values['a3'])

        T1 = float(values['T1'])
        d2 = float(values['d2'])
        d3 = float(values['d3'])

        T1 = T1 / 180.0 * np.pi  # Theta 1 in radians

        DHPT = [[T1, (0.0 / 180.0) * np.pi, 0, a1],
                [(270.0 / 180.0) * np.pi, (270.0 / 180.0) * np.pi, 0, a2 + d2],
                [(0.0 / 180.0) * np.pi, (0.0 / 180.0) * np.pi, 0, a3 + d3]]

        i = 0
        H0_1 = [[np.cos(DHPT[i][0]), -np.sin(DHPT[i][0]) * np.cos(DHPT[i][1]), np.sin(DHPT[i][0]) * np.sin(DHPT[i][1]), DHPT[i][2] * np.cos(DHPT[i][0])],
                [np.sin(DHPT[i][0]), np.cos(DHPT[i][0]) * np.cos(DHPT[i][1]), -np.cos(DHPT[i][0]) * np.sin(DHPT[i][1]), DHPT[i][2] * np.sin(DHPT[i][0])],
                [0, np.sin(DHPT[i][1]), np.cos(DHPT[i][1]), DHPT[i][3]],
                [0, 0, 0, 1]]

        i = 1
        H1_2 = [[np.cos(DHPT[i][0]), -np.sin(DHPT[i][0]) * np.cos(DHPT[i][1]), np.sin(DHPT[i][0]) * np.sin(DHPT[i][1]), DHPT[i][2] * np.cos(DHPT[i][0])],
                [np.sin(DHPT[i][0]), np.cos(DHPT[i][0]) * np.cos(DHPT[i][1]), -np.cos(DHPT[i][0]) * np.sin(DHPT[i][1]), DHPT[i][2] * np.sin(DHPT[i][0])],
                [0, np.sin(DHPT[i][1]), np.cos(DHPT[i][1]), DHPT[i][3]],
                [0, 0, 0, 1]]

        i = 2
        H2_3 = [[np.cos(DHPT[i][0]), -np.sin(DHPT[i][0]) * np.cos(DHPT[i][1]), np.sin(DHPT[i][0]) * np.sin(DHPT[i][1]), DHPT[i][2] * np.cos(DHPT[i][0])],
                [np.sin(DHPT[i][0]), np.cos(DHPT[i][0]) * np.cos(DHPT[i][1]), -np.cos(DHPT[i][0]) * np.sin(DHPT[i][1]), DHPT[i][2] * np.sin(DHPT[i][0])],
                [0, np.sin(DHPT[i][1]), np.cos(DHPT[i][1]), DHPT[i][3]],
                [0, 0, 0, 1]]

        H0_1 = np.matrix(H0_1)
        # print("H0_1 = ")
        # print(H0_1)

        H1_2 = np.matrix(H1_2)
        # print("H1_2 = ")
        # print(H1_2)

        H2_3 = (np.matrix(H2_3))
        # print("H2_3 = ")
        # print(H2_3)

        H0_2 = np.dot(H0_1, H1_2)
        H0_3 = np.dot(H0_2, H2_3)

        # print("H0_2= ")
        # print(np.matrix(H0_2))

        print("H0_3 = ")
        print(np.matrix(H0_3))

#Position Vector

        X0_3 = H0_3[0, 3]
        print("X = ", (X0_3))

        Y0_3 = H0_3[1, 3]
        print("Y = ", Y0_3)

        Z0_3 = H0_3[2, 3]
        print("Z = ", Z0_3)

        disable_J.update(disabled=False)
        disable_IK.update(disabled=False)
        disable_PT.update(disabled=False)

    if event == 'Submit':
        df = df.append(values, ignore_index=True)
        df.to_excel(EXCEL_FILE, index=False)
        sg.popup('Data saved!')

    if event == 'Jacobian Matrix (J)':

        # Linear/Prismatic Vectors

        Z_1 = [[0], [0], [1]]

        # Row 1-3, Column 1

        try:
            H2_3 = np.matrix(H2_3)
        except:
            H2_3 = -1 #NAN
            sg.popup('Warning')
            sg.popup('Restart the GUI then go first to "Click this before Solving Forward Kinematics"!')
            break

        J1a = H2_3[0:3, 0:3]
        J1a = np.dot(J1a, Z_1)
        # print("J1a = ")
        # print(J1a)

        J1b = H0_3[0:3, 3:]
        J1b = np.matrix(J1b)
        # print("J1b = ")
        # print(J1b)

        J1 = [[(J1a[1, 0] * J1b[2, 0]) - (J1a[2, 0] * J1b[1, 0])],
              [(J1a[2, 0] * J1b[0, 0]) - (J1a[0, 0] * J1b[2, 0])],
              [(J1a[0, 0] * J1b[1, 0]) - (J1a[1, 0] * J1b[0, 0])]]
        #print("J1 = ")
        #print(np.matrix(J1))

        # Row 1-3, Column 2

        J2 = H0_1[0:3, 0:3]
        J2 = np.dot(J2, Z_1)
        #print("J2 = ")
        #print(J2)

        # Row 1-3, Column 3

        J3 = H0_2[0:3, 0:3]
        J3 = np.dot(J3, Z_1)
        #print("J3 = ")
        #print(J3)

        # Rotation/Orientation Vectors

        J4 = H2_3[0:3, 0:3]
        J4 = np.dot(J4, Z_1)
        J4 = np.matrix(J4)
        #print("J4 = ")
        #print(J4)

        J5 = [[0], [0], [0]]
        J5 = np.matrix(J5)
        #print("J5 = ")
        #print(J5)

        J6 = [[0], [0], [0]]
        J6 = np.matrix(J6)
        #print("J6 = ")
        #print(J6)

        # Concatenated Jacobian Matrix

        JM1 = np.concatenate((J1, J2, J3), 1)
        # print(JM1)
        JM2 = np.concatenate((J4, J5, J6), 1)
        # print(JM2)

        J = np.concatenate((JM1, JM2), 0)
        #print("J = ")
        #print(J)

        sg.popup('J = ', J)
        DJ = np.linalg.det(JM1)
        if DJ == 0.0 or DJ == -0:
            disable_IV.update(disabled=True)
            sg.popup('Warning: Jacobian Matrix is Non-Invertible!')

        elif DJ != 0.0 or DJ != -0:
            disable_IV.update(disabled=False)


        disable_J.update(disabled=True)
        disable_DetJ.update(disabled=False)
        disable_TJ.update(disabled=False)

    if event == 'Det(J)':

        # SINGULARITIES

        # singularity = Det(J)
        # np.linalg,det(M)
        # Let JMI become the 3x3 position matrix for obtaining the Determinant

        try:
            DJ = np.linalg.det(JM1)
        except:
            DJ = -1 #NAN
            sg.popup('Warning')
            sg.popup('Restart the GUI then go first to "Click this before Solving Forward Kinematics"!')
            break

        DJ = np.linalg.det(JM1)
        #print("DJ = ", DJ)
        sg.popup('DJ = ',DJ)

        if DJ == 0.0 or DJ == -0:
            disable_IV.update(disabled=True)
            sg.popup('Warning: Jacobian Matrix is Non-Invertible!')

    if  event == 'Inverse of J':
        # INVERSE VELOCITY (PEDE DING IV)

        try:
            IJ = np.linalg.inv(JM1)
        except:
            IJ = -1 #NAN
            sg.popup('Warning')
            sg.popup('Restart the GUI then go first to "Click this before Solving Forward Kinematics"!')
            break

        IJ = np.linalg.inv(JM1)
        #print("IV = ")
        # print(IV)
        sg.popup('IJ = ',IJ)

    if event == 'Transpose of J':

        try:
            TJ = np.transpose(JM1)
        except:
            TJ = -1 #NAN
            sg.popup('Warning')
            sg.popup('Restart the GUI then go first to "Click this before Solving Forward Kinematics"!')
            break

        TJ = np.transpose(JM1)
        #print("TJ = ")
        #print(TJ)

        sg.popup('TJ = ',TJ)

window.close()




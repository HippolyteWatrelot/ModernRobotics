import numpy as np
import intervals as I
from sympy.solvers import solve
from sympy import Symbol
import scipy.integrate as integrate
from math import atan2, inf, isinf


"""All of these functions are my personal ideas and personal tricks.
Nothing purely official or correct"""


def IsPolygon(tab):

    """An array of tuples of ordered summits describes a polygon if no segment crosses another one and if first and last
    summits are the same.
    You may not have to pay attention to this function"""

    if not first_drawing_rule_respected(tab):
        print("First drawing rule not respected")
        return False
    if tab[0] != tab[-1] or len(tab) < 4:
        print("Not a polygon 1")
        return False
    for i in range(len(tab) - 2):
        if tab[i + 1][0] != tab[i][0]:
            k1 = (tab[i + 1][1] - tab[i][1]) / (tab[i + 1][0] - tab[i][0])
        else:
            k1 = tab[i][0]
        for j in range(i + 1, len(tab) - 1):
            if tab[i] == tab[j]:
                print("Not a polygon 2")
                return False
            if tab[j + 1][0] != tab[j][0]:
                k2 = (tab[j + 1][1] - tab[j][1]) / (tab[j + 1][0] - tab[j][0])
            else:
                k2 = tab[j][0]
            if tab[i + 1][0] == tab[i][0] and tab[j + 1][0] == tab[j][0]:
                tab1 = [tab[i][1], tab[i + 1][1]]
                tab2 = [tab[j][1], tab[j + 1][1]]
                it1 = I.closed(np.min(tab1), np.max(tab1))
                it2 = I.closed(np.min(tab2), np.max(tab2))
                if k1 == k2:
                    IT = it1 & it2
                    if "()" not in str(IT):
                        print("Not a polygon 3")
                        return False
            if tab[i + 1][0] == tab[i][0] and tab[j + 1][0] != tab[j][0]:
                matx = np.array([tab[j + 1][0], tab[j][0]])
                maty = np.array([tab[i + 1][1], tab[i][1]])
                #x = (tab[i][0] - np.min(matx)) / (np.max(matx) - np.min(matx))
                x = tab[i][0] - np.min(matx)
                y = tab[j][1] + k2 * x
                if np.min(matx) < tab[i][0] < np.max(matx) and np.min(maty) <= y <= np.max(maty):
                    print("Not a polygon 4")
                    return False
            if tab[i + 1][0] != tab[i][0] and tab[j + 1][0] == tab[j][0]:
                matx = np.array([tab[i + 1][0], tab[i][0]])
                maty = np.array([tab[j + 1][1], tab[j][1]])
                #x = (tab[j][0] - np.min(matx)) / (np.max(matx) - np.min(matx))
                x = tab[j][0] - np.min(matx)
                y = tab[i][1] + k1 * x
                if np.min(matx) < tab[j][0] < np.max(matx) and np.min(maty) <= y <= np.max(maty):
                    print("Not a polygon 5")
                    return False
            alpha = Symbol('alpha')
            x = 1 / (1 + alpha ** 2)
            if tab[i + 1][0] != tab[i][0] and tab[j + 1][0] != tab[j][0]:
                it = I.closed(np.min([tab[i][0], tab[i + 1][0]]), np.max([tab[i][0], tab[i + 1][0]]))
                It = I.closed(np.min([tab[j][0], tab[j + 1][0]]), np.max([tab[j][0], tab[j + 1][0]]))
                It_final = it & It
                inf = It_final.lower
                sup = It_final.upper
                T1 = [tab[i][1], tab[i + 1][1]]
                T2 = [tab[j][1], tab[j + 1][1]]
                q1 = np.min([tab[i][0], tab[i + 1][0]])
                n1 = np.argmin([tab[i][0], tab[i + 1][0]])
                q2 = np.min([tab[j][0], tab[j + 1][0]])
                n2 = np.argmin([tab[j][0], tab[j + 1][0]])
                y1 = T1[int(n1)]
                y2 = T2[int(n2)]
                if k1 == k2 and "()" not in str(It_final) and k1 * (inf - q1) + y1 == k2 * (inf - q2) + y2:
                    print("Not a polygon 6: flat angle detected")
                    return False
                if "()" not in str(It_final):
                    s = solve((k1 - k2) * (x * (sup - inf)) + y1 + k1 * (inf - q1) - y2 - k2 * (inf - q2), alpha)
                    if s and 1 / (1 + s[0] ** 2) not in [0, 1] and 1 / (1 + s[1] ** 2) not in [0, 1]:
                        if "I" not in str(s) and inf <= inf + (1 / (1 + s[0] ** 2)) * (sup - inf) <= sup:
                            print("Not a polygon 7")
                            return False
    #print("It's a polygon !")
    return True



def Contact_Placing(tab):
    """Return the coordinates of all the desired contact nodes according to the polygon tab
    It also returns the normal vectors for each of these points of contact"""

    if not IsPolygon(tab):
        return False
    List = []
    Normals = []
    k = 0
    for i in range(len(tab) - 1):
        Arr = [tab[i], tab[i + 1]]
        print("SEGMENT n°", i + 1)
        contacts = input("Type number of contacts on segment:\n")
        memory = []
        if contacts != 0:
            for j in range(int(contacts)):
                contact = -1
                while float(contact) not in I.closedopen(0, 100) and float(contact) not in memory:
                    print("contact n°", j + 1)
                    contact = input("Type percentage position on segment\n")
                    memory.append(float(contact))
                x = Arr[0][0] + float(contact) * (Arr[1][0] - Arr[0][0]) / 100
                y = Arr[0][1] + float(contact) * (Arr[1][1] - Arr[0][1]) / 100
                X1 = tab[i + 1][0] - tab[i][0]
                Y1 = tab[i + 1][1] - tab[i][1]
                if float(contact) != 0:
                    """In this condition, we place a contact point on the actual polygon side which is not a summit"""
                    Normals.append(atan2(-X1, Y1))
                    List.append((x, y))
                    k += 1
                else:
                    if float(contact) == 0 and i >= 1:
                        """In this condition, we place a contact point on a summit"""
                        theta_i = atan2(tab[i + 1][1] - tab[i][1], tab[i + 1][0] - tab[i][0])
                        theta_ind = atan2(tab[i][1] - tab[i - 1][1], tab[i][0] - tab[i - 1][0])
                        Dtheta = theta_i - theta_ind
                        if Dtheta < 0:
                            Dtheta += 2 * np.pi
                        X2 = tab[i][0] - tab[i - 1][0]
                        Y2 = tab[i][1] - tab[i - 1][1]
                        if Dtheta < np.pi:
                            """In this condition, we place a contact point on a "well summit", so for this single
                            contact point, we have actually 2 contacts at this same point and so 2 normals relative
                            to actual side and previous side"""
                            Normals.append(atan2(-X2, Y2))
                            List.append((x, y))
                            Normals.append(atan2(-X1, Y1))
                            List.append((x, y))
                            k += 2
                        else:
                            """Choose your orientation -> In this condition, we place a contact point on a convex 
                            summit so the associated normal vector orientation can be between actual side normal
                            orientation and previous side normal orientation"""
                            theta_1 = atan2(-X1, Y1)
                            theta_2 = atan2(-X2, Y2)
                            if theta_2 < 0 <= theta_1:
                                theta_2 += 2 * np.pi
                                angle = theta_1 - 1
                                while not theta_1 <= float(angle) <= theta_2:
                                    print("Choose in", [theta_1, theta_2])
                                    angle = float(input("Type Angle Value:\n"))
                                if angle > np.pi:
                                    angle -= 2 * np.pi
                            else:
                                angle = theta_1 - 1
                                while not theta_1 <= float(angle) <= theta_2:
                                    print("Choose in", [theta_1, theta_2])
                                    angle = float(input("Type Angle Value:\n"))
                            Normals.append(float(angle))
                            List.append((x, y))
                            k += 1
                    else:
                        if float(contact) == 0 and i == 0:
                            """This is just the case where we place a contact point on the first summit"""
                            X1 = tab[-1][0] - tab[-2][0]
                            Y1 = tab[-1][1] - tab[-2][1]
                            X2 = tab[1][0] - tab[0][0]
                            Y2 = tab[1][1] - tab[0][1]
                            theta_last = atan2(Y1, X1)
                            theta_first = atan2(Y2, X2)
                            Dtheta = theta_first - theta_last
                            if Dtheta < 0:
                                Dtheta += 2 * np.pi
                            if Dtheta > np.pi:
                                """convex summit"""
                                theta_1 = atan2(-X1, Y1)
                                theta_2 = atan2(-X2, Y2)
                                if theta_1 < 0 <= theta_2:
                                    theta_1 += 2 * np.pi
                                    angle = theta_2 - 1
                                    while not theta_2 <= float(angle) <= theta_1:
                                        print("Choose in", [theta_2, theta_1])
                                        angle = float(input("Type Angle Value:\n"))
                                    if angle > np.pi:
                                        angle -= 2 * np.pi
                                else:
                                    angle = theta_2 - 1
                                    while not theta_2 <= float(angle) <= theta_1:
                                        print("Choose in", [theta_2, theta_1])
                                        angle = float(input("Type Angle Value:\n"))
                                Normals.append(angle)
                                List.append(tab[0])
                                k += 1
                            else:
                                """well summit"""
                                List.append(tab[0])
                                List.append(tab[0])
                                Normals.append(atan2(-X1, Y1))
                                Normals.append(atan2(-X2, Y2))
                                k += 2
    if k < 4:
        print("Not enough points of contact")
    return List, Normals



def good_contact(figure1, figure2):

    """This function simply verifies if two polygons touch each other by at least one node or one surface without
    letting one penetrate the other.
    It also returns the contacts coordinates"""

    k = 0
    tab1 = []
    tab2 = []
    It1 = []
    It2 = []
    It1_ord = []
    It2_ord = []
    contacts = []
    assert len(figure1) >= 4
    assert len(figure2) >= 4
    bad_contacts = unsufficient_contacts(figure1, figure2)
    if figure1 == figure2:
        print("same figures")
        return False
    for n in range(len(figure1) - 1):
        ord_tab = [figure1[n][1], figure1[n + 1][1]]
        tab1.append(((figure1[n][0] + figure1[n + 1][0]) / 2, (figure1[n][1] + figure1[n + 1][1]) / 2))
        inf = np.min([figure1[n][0], figure1[n + 1][0]])
        sup = np.max([figure1[n][0], figure1[n + 1][0]])
        inf_ind = np.argmin([figure1[n][0], figure1[n + 1][0]])
        sup_ind = np.argmax([figure1[n][0], figure1[n + 1][0]])
        It1.append(I.closedopen(inf, sup))
        It1_ord.append((ord_tab[int(inf_ind)], ord_tab[int(sup_ind)]))
    for m in range(len(figure2) - 1):
        ord_tab = [figure2[m][1], figure2[m + 1][1]]
        tab2.append(((figure2[m][0] + figure2[m + 1][0]) / 2, (figure2[m][1] + figure2[m + 1][1]) / 2))
        inf = np.min([figure2[m][0], figure2[m + 1][0]])
        sup = np.max([figure2[m][0], figure2[m + 1][0]])
        inf_ind = np.argmin([figure2[m][0], figure2[m + 1][0]])
        sup_ind = np.argmax([figure2[m][0], figure2[m + 1][0]])
        It2.append(I.closedopen(inf, sup))
        It2_ord.append((ord_tab[int(inf_ind)], ord_tab[int(sup_ind)]))
    indice = 0
    for n in range(len(tab1)):
        counter1 = 0
        counter2 = 0
        for m in range(len(tab2)):
            if tab1[n][0] in It2[m]:
                if m == 0:
                    Itb = I.open((It2[-1] | It2[m]).lower, (It2[-1] | It2[m]).upper)
                else:
                    Itb = I.open((It2[m - 1] | It2[m]).lower, (It2[m - 1] | It2[m]).upper)
                if It2[m].lower != It2[m].upper:
                    k = (It2_ord[m][1] - It2_ord[m][0]) / (It2[m].upper - It2[m].lower)
                    alt = It2_ord[m][0] + k * (tab1[n][0] - It2[m].lower)
                    if tab1[n][1] >= alt and tab1[n][0] not in Itb:
                        counter1 += 1
                    if tab1[n][1] <= alt and tab1[n][0] not in Itb:
                        counter2 += 1
        if counter1 % 2 == 0 or counter2 % 2 == 0:
            indice = 1
    if indice == 0:
        print("Penetration detected (1)")
        return -1, contacts
    indice = 0
    for n in range(len(tab2)):
        counter1 = 0
        counter2 = 0
        for m in range(len(tab1)):
            if tab2[n][0] in It1[m]:
                if m == 0:
                    Itb = I.open((It1[-1] | It1[m]).lower, (It1[-1] | It1[m]).upper)
                else:
                    Itb = I.open((It1[m - 1] | It1[m]).lower, (It1[m - 1] | It1[m]).upper)
                if It1[m].lower != It1[m].upper:
                    k = (It1_ord[m][1] - It1_ord[m][0]) / (It1[m].upper - It1[m].lower)
                    alt = It1_ord[m][0] + k * (tab2[n][0] - It1[m].lower)
                    if tab2[n][1] >= alt and tab2[n][0] not in Itb:
                        counter1 += 1
                    if tab2[n][1] <= alt and tab2[n][0] not in Itb:
                        counter2 += 1
        if counter1 % 2 == 0 or counter2 % 2 == 0:
            indice = 1
    if indice == 0:
        print("Penetration detected (2)")
        return -1, contacts
    for i in range(len(figure1) - 1):
        for j in range(len(figure2) - 1):
            if figure1[i + 1][0] != figure1[i][0] and figure2[j + 1][0] != figure2[j][0]:
                k1 = (figure1[i + 1][1] - figure1[i][1]) / (figure1[i + 1][0] - figure1[i][0])
                k2 = (figure2[j + 1][1] - figure2[j][1]) / (figure2[j + 1][0] - figure2[j][0])
                inf_1 = np.min([figure1[i][0], figure1[i + 1][0]])
                sup_1 = np.max([figure1[i][0], figure1[i + 1][0]])
                inf_2 = np.min([figure2[j][0], figure2[j + 1][0]])
                sup_2 = np.max([figure2[j][0], figure2[j + 1][0]])
                interval = I.closed(inf_1, sup_1) & I.closed(inf_2, sup_2)
                if "()" not in str(interval):
                    inf = interval.lower
                    sup = interval.upper
                    x = Symbol("x")
                    absc = sup - inf
                    if absc != 0:
                        T1 = [figure1[i][1], figure1[i + 1][1]]
                        T2 = [figure2[j][1], figure2[j + 1][1]]
                        q1 = np.min([figure1[i][0], figure1[i + 1][0]])
                        q2 = np.min([figure2[j][0], figure2[j + 1][0]])
                        Q1 = np.max([figure1[i][0], figure1[i + 1][0]])
                        Q2 = np.max([figure2[j][0], figure2[j + 1][0]])
                        n1 = np.argmin([figure1[i][0], figure1[i + 1][0]])
                        n2 = np.argmin([figure2[j][0], figure2[j + 1][0]])
                        y1 = T1[int(n1)]
                        y2 = T2[int(n2)]
                        seg1 = inf + k1 * x * absc + y1 + k1 * (inf - q1)
                        seg2 = inf + k2 * x * absc + y2 + k2 * (inf - q2)
                        s = solve(seg1 - seg2, x)
                        if k1 != k2:
                            if s[0] in I.open(0, 1):
                                print("Penetration detected (3)")
                                return -1, contacts
                            else:
                                #if s[0] == 0 and (inf == q1 or inf == q2) or s[0] == 1 and (sup == Q1 or sup ==Q2):
                                if s[0] == 0 and inf == q1:
                                    if (q1, y1) not in contacts and (q1, y1) not in bad_contacts:
                                        k = 1
                                        contacts.append((q1, y1))
                                        #print("1")
                                elif s[0] == 0 and inf == q2:
                                    if (q2, y2) not in contacts and (q2, y2) not in bad_contacts:
                                        k = 1
                                        contacts.append((q2, y2))
                                        #print("2")
                                elif s[0] == 1 and sup == Q1:
                                    if (Q1, y1 + k1 * (Q1 - q1)) not in contacts and \
                                            (Q1, y1 + k1 * (Q1 - q1)) not in bad_contacts:
                                        k = 1
                                        contacts.append((Q1, y1 + k1 * (Q1 - q1)))
                                        #print("3")
                                elif s[0] == 1 and sup == Q2:
                                    if (Q2, y2 + k2 * (Q2 - q2)) not in contacts and \
                                            (Q2, y2 + k2 * (Q2 - q2)) not in bad_contacts:
                                        k = 1
                                        contacts.append((Q2, y2 + k2 * (Q2 - q2)))
                                        #print("4")
                        else:
                            if k1 == k2 and figure1[i][1] == figure2[j][1]:
                                if "()" not in str(interval):
                                    y1 = figure1[i][1] + k1 * (interval.lower - figure1[i][0])
                                    y2 = figure1[i][1] + k1 * (interval.upper - figure1[i][0])
                                    if (interval.lower, y1) not in contacts and \
                                            (interval.lower, y1) not in bad_contacts:
                                        k = 1
                                        contacts.append((interval.lower, y1))
                                        #print("5")
                                    if (interval.upper, y2) not in contacts and \
                                            (interval.upper, y2) not in bad_contacts:
                                        k = 1
                                        contacts.append((interval.upper, y2))
                                        #print("6")
            else:
                if figure1[i + 1][0] == figure1[i][0] and figure2[j + 1][0] != figure2[j][0]:
                    k2 = (figure2[j + 1][1] - figure2[j][1]) / (figure2[j + 1][0] - figure2[j][0])
                    inf_y = np.min([figure1[i][1], figure1[i + 1][1]])
                    sup_y = np.max([figure1[i][1], figure1[i + 1][1]])
                    interval_y = I.open(inf_y, sup_y)
                    Tab = [figure2[j][0], figure2[j + 1][0]]
                    int_abs = I.open(np.min(Tab), np.max(Tab))
                    val = figure2[j][1] + k2 * (figure1[i][0] - figure2[j][0])
                    if val in interval_y and figure1[i][0] in int_abs:
                        print("Penetration detected (4)")
                        return -1, contacts
                    else:
                        #if val in [inf_y, sup_y] or figure2[j][0] == figure1[i][0] or figure2[j + 1][0] == figure1[i][0]:
                        if val == inf_y and figure1[i][0] in int_abs:
                            if (figure1[i][0], inf_y) not in contacts and (figure1[i][0], inf_y) not in bad_contacts:
                                k = 1
                                contacts.append((figure1[i][0], inf_y))
                                #print("7")
                        elif val == sup_y and figure1[i][0] in int_abs:
                            if (figure1[i][0], sup_y) not in contacts and (figure1[i][0], sup_y) not in bad_contacts:
                                k = 1
                                contacts.append((figure1[i][0], sup_y))
                                #print("8")
                        elif figure2[j][0] == figure1[i][0] and figure2[j][1] == figure1[i][1]:
                            if (figure1[i][0], figure2[j][1]) not in contacts and \
                                    (figure1[i][0], figure2[j][1]) not in bad_contacts:
                                k = 1
                                contacts.append((figure1[i][0], figure2[j][1]))
                                #print("9")
                        elif figure2[j + 1][0] == figure1[i][0] and figure2[j + 1][1] == figure1[i][1]:
                            if (figure1[i][0], figure2[j + 1][1]) not in contacts and \
                                    (figure1[i][0], figure2[j + 1][1]) not in bad_contacts:
                                k = 1
                                contacts.append((figure1[i][0], figure2[j + 1][1]))
                                #print("10")
                else:
                    if figure1[i + 1][0] != figure1[i][0] and figure2[j + 1][0] == figure2[j][0]:
                        k1 = (figure1[i + 1][1] - figure1[i][1]) / (figure1[i + 1][0] - figure1[i][0])
                        inf_y = np.min([figure2[j][1], figure2[j + 1][1]])
                        sup_y = np.max([figure2[j][1], figure2[j + 1][1]])
                        interval_y = I.open(inf_y, sup_y)
                        Tab = [figure1[i][0], figure1[i + 1][0]]
                        int_abs = I.open(np.min(Tab), np.max(Tab))
                        val = figure1[i][1] + k1 * (figure2[j][0] - figure1[i][0])
                        if val in interval_y and figure2[j][0] in int_abs:
                            print("Penetration detected (5)")
                            return -1, contacts
                        else:
                            #if val in [inf_y, sup_y] or figure1[i][0] == figure2[j][0] or figure1[i + 1][0] == figure2[j][0]:
                            if val == inf_y and figure2[j][0] in int_abs:
                                if (figure2[j][0], inf_y) not in contacts and \
                                        (figure2[j][0], inf_y) not in bad_contacts:
                                    k = 1
                                    contacts.append((figure2[j][0], inf_y))
                                    #print("11")
                            elif val == sup_y and figure2[j][0] in int_abs:
                                if (figure2[j][0], sup_y) not in contacts and \
                                        (figure2[j][0], sup_y) not in bad_contacts:
                                    k = 1
                                    contacts.append((figure2[j][0], sup_y))
                                    #print("12")
                            elif figure1[i][0] == figure2[j][0] and figure1[i][1] == figure2[j][1]:
                                if (figure2[j][0], figure1[i][1]) not in contacts and \
                                        (figure2[j][0], figure1[i][1]) not in bad_contacts:
                                    k = 1
                                    contacts.append((figure2[j][0], figure1[i][1]))
                                    #print("13")
                            elif figure1[i + 1][0] == figure2[j][0] and figure1[i + 1][1] == figure2[j][1]:
                                if (figure2[j][0], figure1[i + 1][1]) not in contacts and \
                                        (figure2[j][0], figure1[i + 1][1]) not in bad_contacts:
                                    k = 1
                                    contacts.append((figure2[j][0], figure1[i + 1][1]))
                                    #print("14")
                    else:
                        if figure1[i + 1][0] == figure1[i][0] and figure2[j + 1][0] == figure2[j][0] and \
                                figure1[i][0] == figure2[j][0]:
                            inf_1 = np.min([figure1[i][1], figure1[i + 1][1]])
                            sup_1 = np.max([figure1[i][1], figure1[i + 1][1]])
                            inf_2 = np.min([figure2[j][1], figure2[j + 1][1]])
                            sup_2 = np.max([figure2[j][1], figure2[j + 1][1]])
                            interval_Y = I.closed(inf_1, sup_1) & I.closed(inf_2, sup_2)
                            if "()" not in str(interval_Y):
                                if (figure1[i][0], interval_Y.lower) not in contacts and \
                                        (figure1[i][0], interval_Y.lower) not in bad_contacts:
                                    k = 1
                                    contacts.append((figure1[i][0], interval_Y.lower))
                                    #print("15")
                                if (figure1[i][0], interval_Y.upper) not in contacts and \
                                        (figure1[i][0], interval_Y.upper) not in bad_contacts:
                                    k = 1
                                    contacts.append((figure1[i][0], interval_Y.upper))
                                    #print("16")
    if k == 1:
        #print("Good contact(s)")
        return 1, contacts
    else:
        #print("No contact")
        return 0, contacts



def mass_and_center_of_gravity(figure):

    """This function calculates the surface mass of a polygon (with 1 as mass per surface unit) and the coordinates of
    its center of gravity"""

    assert IsPolygon(figure)
    xtab = []
    ytab = []
    dtype1 = [('abs', float), ('ord', float)]
    fig = np.array(figure, dtype=dtype1)
    np.sort(fig, order='abs')
    steps = []
    for i in range(len(figure)):
        xtab.append(figure[i][0])
        ytab.append(figure[i][1])
        if figure[i][0] not in steps:
            steps.append(figure[i][0])
    steps = np.sort(steps)
    mass_coefficients = []
    solutions_coordinates = []
    for i in range(len(steps) - 1):
        cog_part_coordinates = []
        val = (steps[i] + steps[i + 1]) / 2
        counter = 0
        for j in range(len(figure) - 1):
            tab = [figure[j][0], figure[j + 1][0]]
            it = I.open(np.min(tab), np.max(tab))
            args = [np.argmin(tab), np.argmax(tab)]
            if val in it:
                counter += 1
                k = (figure[j + 1][1] - figure[j][1]) / (figure[j + 1][0] - figure[j][0])
                yl = figure[j + int(args[0])][1] + (steps[i] - figure[j + int(args[0])][0]) * k
                yr = figure[j + int(args[0])][1] + (steps[i + 1] - figure[j + int(args[0])][0]) * k
                node1 = list(figure[j + int(args[0])])
                node2 = list(figure[j + int(args[1])])
                middle = (yl + yr) / 2
                cog_part_coordinates.append((node1, node2, middle))
        assert counter % 2 == 0
        dtype2 = [('Node1', list), ('Node2', list), ('Middle', float)]
        cog_part_coordinates = np.array(cog_part_coordinates, dtype=dtype2)
        cog_part_coordinates = np.sort(cog_part_coordinates, order='Middle')
        for j in range(len(cog_part_coordinates) // 2):
            duo1 = [cog_part_coordinates[2 * j][0], cog_part_coordinates[2 * j][1]]
            middle1 = cog_part_coordinates[2 * j][-1]
            duo2 = [cog_part_coordinates[2 * j + 1][0], cog_part_coordinates[2 * j + 1][1]]
            middle2 = cog_part_coordinates[2 * j + 1][-1]
            k1 = (duo1[1][1] - duo1[0][1]) / (duo1[1][0] - duo1[0][0])
            k2 = (duo2[1][1] - duo2[0][1]) / (duo2[1][0] - duo2[0][0])
            deltaX = steps[i + 1] - steps[i]
            area = abs(middle1 - middle2) * deltaX
            yl_middle = (duo1[0][1] + k1 * (steps[i] - duo1[0][0]) + duo2[0][1] + k2 * (steps[i] - duo2[0][0])) / 2
            yr_middle = (duo1[0][1] + k1 * (steps[i + 1] - duo1[0][0]) + duo2[0][1] + k2 * (steps[i + 1] - duo2[0][0])) / 2
            k_middle = (yr_middle - yl_middle) / (steps[i + 1] - steps[i])
            fx = lambda x: x * abs(duo1[0][1] + k1 * (x - duo1[0][0]) - duo2[0][1] - k2 * (x - duo2[0][0]))
            X1 = steps[i]
            X2 = steps[i + 1]
            xsol = integrate.quad(fx, X1, X2)[0] / area
            ysol = yl_middle + k_middle * (xsol - steps[i])
            solutions_coordinates.append((xsol, ysol))
            mass_coefficients.append(area)
    xsolution = 0
    ysolution = 0
    mass = np.sum(mass_coefficients)
    for i in range(len(mass_coefficients)):
        xsolution += solutions_coordinates[i][0] * mass_coefficients[i] / mass
        ysolution += solutions_coordinates[i][1] * mass_coefficients[i] / mass

    return mass, (xsolution, ysolution)#, mass_coefficients, solutions_coordinates


#fig = [(0, 0), (0, 2), (2, 2), (3, 3), (2, 0), (0, 0)]
#IsPolygon(fig)
#absc = []
#ord = []
#for i in range(len(fig)):
#    absc.append(fig[i][0])
#    ord.append(fig[i][1])
#exe = mass_and_center_of_gravity(fig)
#mass = exe[0]
#cog = exe[1]
#mass_coeff = exe[2]
#sc = exe[3]
#print("mass of polygon", mass)
#print("center of gravity coordinates", cog)
#print("mass coefficients", mass_coeff)
#print("coordinates", sc)
#plt.plot(absc, ord)
#plt.plot([cog[0]], [cog[1]], "ro")
#plt.show()


def rigid_bodies_assembly(assembly, ground=0):

    """This function simply tests if an assembly of figures does show a "full assembly" (Only one resulting block with
    coherent contacts) and that there is no case of one polygon penetrating another one.
    - ground: ordinate of the ground where the assembly is laying"""

    figures_test = []
    ords = []
    figure = assembly[0]
    for i in range(len(assembly)):
        figure = assembly[i]
        for j in range(len(figure)):
            ords.append(figure[j][1])
        if not IsPolygon(figure) or np.min(ords) < ground:
            print("Error")
            return False
    if ground not in ords:
        return "BAD ASSEMBLY: Assembly not in contact with the ground ! "
    figures_test.append(figure)
    for i in range(1, len(assembly)):
        k = 0
        figure = assembly[i]
        if not IsPolygon(figure):
            print("Error !")
            return False
        for j in range(len(assembly)):
            if i != j:
                figure_test = assembly[j]
                test = good_contact(figure, figure_test)
                if test[0] == 1:
                    k = 1
                else:
                    if test[0] == -1:
                        print("Error !")
                        return False
        if k == 0:
            print("BAD ASSEMBLY")
            return False
    print("GOOD ASSEMBLY")
    return True



def contacts_vectors(assembly, ground=0):

    """This function describes for all contacts coordinates the associated contacts vectors."""

    assert rigid_bodies_assembly(assembly)
    all_contacts = []
    contacts_vectors_angles = []
    coherent_contacts = []
    for i in range(len(assembly) - 1):
        for j in range(i + 1, len(assembly)):
            contacts = good_contact(assembly[i], assembly[j])[1]
            for k in range(len(contacts)):
                if contacts[k] not in all_contacts:
                    all_contacts.append(contacts[k])
    for i in range(len(assembly)):
        for indice in range(len(assembly[i])):
            if assembly[i][indice][1] == ground:
                coherent_contacts.append(assembly[i][indice])
                contacts_vectors_angles.append((- np.pi / 2, np.pi / 2))
                if assembly[i][indice] not in all_contacts:
                    all_contacts.append(assembly[i][indice])
    for i in range(len(all_contacts)):
        for j in range(len(assembly) - 1):
            for n in range(len(assembly[j]) - 1):
                tab_abs1 = [assembly[j][n][0], assembly[j][n + 1][0]]
                tab_ord1 = [assembly[j][n][1], assembly[j][n + 1][1]]
                segment_abs1 = I.closed(np.min(tab_abs1), np.max(tab_abs1))
                segment_ord1 = I.closed(np.min(tab_ord1), np.max(tab_ord1))
                if assembly[j][n][0] != assembly[j][n + 1][0]:
                    k1 = (tab_ord1[1] - tab_ord1[0]) / (tab_abs1[1] - tab_abs1[0])
                else:
                    k1 = inf
                if all_contacts[i][0] in segment_abs1 and all_contacts[i][1] in segment_ord1:
                    for k in range(j + 1, len(assembly)):
                        for m in range(len(assembly[k]) - 1):
                            tab_abs2 = [assembly[k][m][0], assembly[k][m + 1][0]]
                            tab_ord2 = [assembly[k][m][1], assembly[k][m + 1][1]]
                            segment_abs2 = I.closed(np.min(tab_abs2), np.max(tab_abs2))
                            segment_ord2 = I.closed(np.min(tab_ord2), np.max(tab_ord2))
                            if assembly[k][m][0] != assembly[k][m + 1][0]:
                                k2 = (tab_ord2[1] - tab_ord2[0]) / (tab_abs2[1] - tab_abs2[0])
                            else:
                                k2 = inf
                            if all_contacts[i][0] in segment_abs2 and all_contacts[i][1] in segment_ord2:
                                It_x = segment_abs1 & segment_abs2
                                It_y = segment_ord1 & segment_ord2
                                if It_x.lower != It_x.upper and k1 == k2 and not isinf(k1):
                                    coherent_contacts.append(all_contacts[i])
                                    if assembly[j][n][0] != assembly[j][n + 1][0]:
                                        angle1 = atan2(assembly[j][n + 1][1] - assembly[j][n][1], assembly[j][n + 1][1]
                                                       - assembly[j][n][1]) + np.pi / 2
                                        angle2 = atan2(assembly[j][n + 1][1] - assembly[j][n][1], assembly[j][n + 1][1]
                                                       - assembly[j][n][1]) - np.pi / 2
                                        contacts_vectors_angles.append((angle1, angle2))
                                    elif assembly[j][n][0] == assembly[j][n + 1][0] \
                                            and assembly[k][m][0] != assembly[k][m + 1][0]:
                                        angle1 = atan2(assembly[k][m + 1][1] - assembly[k][m][1], assembly[k][m + 1][1]
                                                       - assembly[k][m][1]) + np.pi / 2
                                        angle2 = atan2(assembly[k][m + 1][1] - assembly[k][m][1], assembly[k][m + 1][1]
                                                       - assembly[k][m][1]) - np.pi / 2
                                        contacts_vectors_angles.append((angle1, angle2))
                                elif It_x.lower == It_x.upper and It_y.lower != It_y.upper:
                                    coherent_contacts.append(all_contacts[i])
                                    angle1 = np.pi
                                    angle2 = 0
                                    contacts_vectors_angles.append((angle1, angle2))
    cc = []
    cva = []
    both = []
    for ind in range(len(coherent_contacts)):
        if (coherent_contacts[ind], contacts_vectors_angles[ind]) not in both:
            cc.append(coherent_contacts[ind])
            cva.append(contacts_vectors_angles[ind])
            both.append((coherent_contacts[ind], contacts_vectors_angles[ind]))
    return cc, cva, all_contacts



def unsufficient_contacts(figure1, figure2):

    """This function (used in good_contacts function) has the role to eliminate the contacts formed by two convex
    summits of two polygons (without surface contact), indeed: in this case, their is full instability for such a
    contact"""

    bad_contacts = []
    for n in range(len(figure1) - 1):
        for m in range(len(figure2) - 1):
            if figure1[n] == figure2[m]:
                test = False
                if n >= 1:
                    infordn = figure1[n - 1][1]
                    infabsn = figure1[n - 1][0]
                else:
                    infordn = figure1[-2][1]
                    infabsn = figure1[-2][0]
                if m >= 1:
                    infordm = figure2[m - 1][1]
                    infabsm = figure2[m - 1][0]
                else:
                    infordm = figure2[-2][1]
                    infabsm = figure2[-2][0]
                if figure1[n][0] != infabsn:
                    k1 = (figure1[n][1] - infordn) / (figure1[n][0] - infabsn)
                    if figure2[m][0] != infabsm:
                        k2 = (figure2[m][1] - infordm) / (figure2[m][0] - infabsm)
                        tab1 = [figure1[n][0], infabsn]
                        tab2 = [figure2[m][0], infabsm]
                        It = I.open(np.min(tab1), np.max(tab1)) & I.open(np.min(tab2), np.max(tab2))
                        if k1 == k2 and "()" not in str(It):
                            test = True
                    elif figure2[m + 1][0] != figure2[m][0]:
                        k2 = (figure2[m + 1][1] - figure2[m][1]) / (figure2[m + 1][0] - figure2[m][0])
                        tab1 = [figure1[n][0], infabsn]
                        tab2 = [figure2[m + 1][0], figure2[m][0]]
                        It = I.open(np.min(tab1), np.max(tab1)) & I.open(np.min(tab2), np.max(tab2))
                        if k1 == k2 and "()" not in str(It):
                            test = True
                else:
                    if figure2[m][0] == infabsm:
                        tab1 = [figure1[n][1], infordn]
                        tab2 = [figure2[m][1], infordm]
                        It = I.open(np.min(tab1), np.max(tab1)) & I.open(np.min(tab2), np.max(tab2))
                        if "()" not in str(It):
                            test = True
                    elif figure2[m + 1][0] == figure2[m][0]:
                        tab1 = [figure1[n][1], infordn]
                        tab2 = [figure2[m + 1][1], figure2[m][1]]
                        It = I.open(np.min(tab1), np.max(tab1)) & I.open(np.min(tab2), np.max(tab2))
                        if "()" not in str(It):
                            test = True
                if figure1[n + 1][0] != figure1[n][0]:
                    k1 = (figure1[n + 1][1] - figure1[n][1]) / (figure1[n + 1][0] - figure1[n][0])
                    if figure2[m][0] != infabsm:
                        k2 = (figure2[m][1] - infordm) / (figure2[m][0] - infabsm)
                        tab1 = [figure1[n + 1][0], figure1[n][0]]
                        tab2 = [figure2[m][0], infabsm]
                        It = I.open(np.min(tab1), np.max(tab1)) & I.open(np.min(tab2), np.max(tab2))
                        if k1 == k2 and "()" not in str(It):
                            test = True
                    elif figure2[m + 1][0] != figure2[m][0]:
                        k2 = (figure2[m + 1][1] - figure2[m][1]) / (figure2[m + 1][0] - figure2[m][0])
                        tab1 = [figure1[n + 1][0], figure1[n][0]]
                        tab2 = [figure2[m + 1][0], figure2[m][0]]
                        It = I.open(np.min(tab1), np.max(tab1)) & I.open(np.min(tab2), np.max(tab2))
                        if k1 == k2 and "()" not in str(It):
                            test = True
                else:
                    if figure2[m][0] == infabsm:
                        tab1 = [figure1[n + 1][1], figure1[n][1]]
                        tab2 = [figure2[m][1], infordm]
                        It = I.open(np.min(tab1), np.max(tab1)) & I.open(np.min(tab2), np.max(tab2))
                        if "()" not in str(It):
                            test = True
                    elif figure2[m + 1][0] == figure2[m][0]:
                        tab1 = [figure1[n + 1][1], figure1[n][1]]
                        tab2 = [figure2[m + 1][1], figure2[m][1]]
                        It = I.open(np.min(tab1), np.max(tab1)) & I.open(np.min(tab2), np.max(tab2))
                        if "()" not in str(It):
                            test = True
                if not test:
                    angle1plus = atan2(figure1[n + 1][1] - figure1[n][1],
                                       figure1[n + 1][0] - figure1[n][0])
                    angle1 = atan2(figure1[n][1] - infordn, figure1[n][0] - infabsn)
                    angle2plus = atan2(figure2[m + 1][1] - figure2[m][1],
                                    figure2[m + 1][0] - figure2[m][0])
                    angle2 = atan2(figure2[m][1] - infordm, figure2[m][0] - infabsm)
                    if angle1plus < 0:
                        angle1plus += 2 * np.pi
                    if angle1 < 0:
                        angle1 += 2 * np.pi
                    if angle2plus < 0:
                        angle2plus += 2 * np.pi
                    if angle2 < 0:
                        angle2 += 2 * np.pi
                    Delta1 = angle1plus - angle1
                    if Delta1 < 0:
                        Delta1 += 2 * np.pi
                    Delta2 = angle2plus - angle2
                    if Delta2 < 0:
                        Delta2 += 2 * np.pi
                    if Delta1 in I.open(np.pi, 2 * np.pi) and Delta2 in I.open(np.pi, 2 * np.pi):
                        bad_contacts.append(figure1[n])
    return bad_contacts

#figure1 = [(0, 0), (0, 1), (1, 1), (1, 0), (0, 0)]
#figure2 = [(1, 1), (1, 2), (2, 2), (2, 1), (1, 1)]
#print(unsufficient_contacts(figure1, figure2))


def first_drawing_rule_respected(figure):

    absc = []
    ord = []
    for i in range(len(figure) - 1):
        absc.append(figure[i][0])
        ord.append(figure[i][1])
    ind_left, ind_right, ind_down, ind_up = np.argmin(absc), np.argmax(absc), np.argmin(ord), np.argmax(ord)
    xleft, yleft = np.min(absc), ord[int(ind_left)]
    left = (xleft, yleft)
    figure2 = []
    for i in range(len(figure)):
        if i != ind_left:
            figure2.append(figure[i])
    absc2, ord2 = [], []
    for i in range(len(figure2) - 1):
        absc2.append(figure2[i][0])
        ord2.append(figure2[i][1])
    ind_up2 = np.argmax(ord2)
    xup, yup = absc[int(ind_up)], np.max(ord)
    up = (xup, yup)
    if left == up and yleft == np.max(ord2):
        xup, yup = absc2[int(ind_up2)], np.max(ord2)
        up = (xup, yup)
    xright, yright = np.max(absc), ord[int(ind_right)]
    right = (xright, yright)
    figure2 = []
    for i in range(len(figure)):
        if i != ind_right:
            figure2.append(figure[i])
    absc2, ord2 = [], []
    for i in range(len(figure2) - 1):
        absc2.append(figure2[i][0])
        ord2.append(figure2[i][1])
    ind_down2 = np.argmin(ord2)
    xdown, ydown = absc[int(ind_down)], np.min(ord)
    down = (xdown, ydown)
    if right == down and yright == np.min(ord2):
        xdown, ydown = absc2[int(ind_down2)], np.min(ord2)
        down = (xdown, ydown)
    #print("LURD", left, up, right, down)
    L, U, R, D = 0, 0, 0, 0
    for j in range(len(figure) - 1):
        if left == figure[j]:
            L = j
        elif up == figure[j]:
            U = j
        elif right == figure[j]:
            R = j
        elif down == figure[j]:
            D = j
    Tab1, Tab2, Tab3, Tab4 = np.array([L, U, R, D]), np.array([D, L, U, R]), np.array([R, D, L, U]), \
                             np.array([U, R, D, L])
    #print(Tab1, Tab2, Tab3, Tab4)
    Test1, Test2, Test3, Test4 = np.sort(Tab1), np.sort(Tab2), np.sort(Tab3), np.sort(Tab4)
    #print(Test1, Test2, Test3, Test4)
    if not np.array_equal(Tab1, Test1) and not np.array_equal(Tab2, Test2) and not np.array_equal(Tab3, Test3) \
            and not np.array_equal(Tab1, Test1):
        return False

    return True

#figure1 = [(0, 0), (0, 1), (1, 1), (2, 0.9), (0, 0)]
#figure2 = [(0, 0), (1, 0), (2, -2), (0, 0)]
#print(first_drawing_rule_respected(figure1))
#print(first_drawing_rule_respected(figure2))
#IsPolygon(figure1)
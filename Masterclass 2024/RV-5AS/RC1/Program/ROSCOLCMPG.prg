1 If M_Out(1100)=1 Then GoSub *Compliance Else *Collision
2 '
3 '
4 *Collision
5 GetM 1
6 ColChk On
7 RelM
8 Return
9 '
10 *Compliance
11 GetM 1
12 ColChk Off
13 CmpG 0.1,0.1,0.1,0.3,0.3,0.3,,
14 Cmp Pos, &B111
15 RelM
16 Return
17 '

(define (problem maintenance-scheduling-1-3-25-100-1-2)
 (:domain maintenance-scheduling-domain)
 (:objects d1 d2 d3 d4 d5 d6 d7 d8 d9 d10 d11 d12 d13 d14 d15 d16 d17 d18 d19 d20 d21 d22 d23 d24 d25 d26 - day
   FRA BER HAM - airport
   ap1 ap2 ap3 ap4 ap5 ap6 ap7 ap8 ap9 ap10 ap11 ap12 ap13 ap14 ap15 ap16 ap17 ap18 ap19 ap20 ap21 ap22 ap23 ap24 ap25 ap26 ap27 ap28 ap29 ap30 ap31 ap32 ap33 ap34 ap35 ap36 ap37 ap38 ap39 ap40 ap41 ap42 ap43 ap44 ap45 ap46 ap47 ap48 ap49 ap50 ap51 ap52 ap53 ap54 ap55 ap56 ap57 ap58 ap59 ap60 ap61 ap62 ap63 ap64 ap65 ap66 ap67 ap68 ap69 ap70 ap71 ap72 ap73 ap74 ap75 ap76 ap77 ap78 ap79 ap80 ap81 ap82 ap83 ap84 ap85 ap86 ap87 ap88 ap89 ap90 ap91 ap92 ap93 ap94 ap95 ap96 ap97 ap98 ap99 ap100 - plane)
 (:init
  (today d1)  (today d2)  (today d3)  (today d4)  (today d5)  (today d6)  (today d7)  (today d8)  (today d9)  (today d10)  (today d11)  (today d12)  (today d13)  (today d14)  (today d15)  (today d16)  (today d17)  (today d18)  (today d19)  (today d20)  (today d21)  (today d22)  (today d23)  (today d24)  (today d25)  (at ap1 d4 BER)
  (at ap2 d23 FRA)
  (at ap3 d9 FRA)
  (at ap4 d12 FRA)
  (at ap5 d4 HAM)
  (at ap6 d4 BER)
  (at ap7 d9 HAM)
  (at ap8 d16 FRA)
  (at ap9 d18 FRA)
  (at ap10 d19 FRA)
  (at ap11 d1 FRA)
  (at ap12 d14 HAM)
  (at ap13 d25 BER)
  (at ap14 d4 BER)
  (at ap15 d14 HAM)
  (at ap16 d4 BER)
  (at ap17 d12 FRA)
  (at ap18 d3 BER)
  (at ap19 d1 HAM)
  (at ap20 d8 FRA)
  (at ap21 d12 FRA)
  (at ap22 d15 FRA)
  (at ap23 d11 HAM)
  (at ap24 d4 BER)
  (at ap25 d1 HAM)
  (at ap26 d7 HAM)
  (at ap27 d20 FRA)
  (at ap28 d25 BER)
  (at ap29 d15 HAM)
  (at ap30 d4 FRA)
  (at ap31 d1 HAM)
  (at ap32 d23 FRA)
  (at ap33 d10 HAM)
  (at ap34 d11 HAM)
  (at ap35 d15 HAM)
  (at ap36 d16 FRA)
  (at ap37 d5 HAM)
  (at ap38 d25 FRA)
  (at ap39 d11 BER)
  (at ap40 d19 BER)
  (at ap41 d3 BER)
  (at ap42 d14 HAM)
  (at ap43 d25 FRA)
  (at ap44 d18 BER)
  (at ap45 d18 HAM)
  (at ap46 d19 FRA)
  (at ap47 d5 BER)
  (at ap48 d8 HAM)
  (at ap49 d17 FRA)
  (at ap50 d2 HAM)
  (at ap51 d12 FRA)
  (at ap52 d11 FRA)
  (at ap53 d24 HAM)
  (at ap54 d7 FRA)
  (at ap55 d16 FRA)
  (at ap56 d22 FRA)
  (at ap57 d3 FRA)
  (at ap58 d16 BER)
  (at ap59 d25 HAM)
  (at ap60 d15 HAM)
  (at ap61 d6 HAM)
  (at ap62 d13 BER)
  (at ap63 d1 FRA)
  (at ap64 d2 FRA)
  (at ap65 d17 HAM)
  (at ap66 d4 BER)
  (at ap67 d22 BER)
  (at ap68 d23 BER)
  (at ap69 d2 FRA)
  (at ap70 d7 BER)
  (at ap71 d8 BER)
  (at ap72 d13 BER)
  (at ap73 d15 BER)
  (at ap74 d13 HAM)
  (at ap75 d13 FRA)
  (at ap76 d11 BER)
  (at ap77 d15 HAM)
  (at ap78 d6 HAM)
  (at ap79 d21 FRA)
  (at ap80 d13 BER)
  (at ap81 d14 BER)
  (at ap82 d21 BER)
  (at ap83 d7 FRA)
  (at ap84 d7 BER)
  (at ap85 d25 HAM)
  (at ap86 d6 BER)
  (at ap87 d18 FRA)
  (at ap88 d21 FRA)
  (at ap89 d24 HAM)
  (at ap90 d24 HAM)
  (at ap91 d15 BER)
  (at ap92 d8 FRA)
  (at ap93 d8 BER)
  (at ap94 d22 FRA)
  (at ap95 d1 BER)
  (at ap96 d19 BER)
  (at ap97 d1 FRA)
  (at ap98 d25 HAM)
  (at ap99 d8 BER)
  (at ap100 d19 FRA)
)
  (:goal (and 
 (done ap1)
 (done ap2)
 (done ap3)
 (done ap4)
 (done ap5)
 (done ap6)
 (done ap7)
 (done ap8)
 (done ap9)
 (done ap10)
 (done ap11)
 (done ap12)
 (done ap13)
 (done ap14)
 (done ap15)
 (done ap16)
 (done ap17)
 (done ap18)
 (done ap19)
 (done ap20)
 (done ap21)
 (done ap22)
 (done ap23)
 (done ap24)
 (done ap25)
 (done ap26)
 (done ap27)
 (done ap28)
 (done ap29)
 (done ap30)
 (done ap31)
 (done ap32)
 (done ap33)
 (done ap34)
 (done ap35)
 (done ap36)
 (done ap37)
 (done ap38)
 (done ap39)
 (done ap40)
 (done ap41)
 (done ap42)
 (done ap43)
 (done ap44)
 (done ap45)
 (done ap46)
 (done ap47)
 (done ap48)
 (done ap49)
 (done ap50)
 (done ap51)
 (done ap52)
 (done ap53)
 (done ap54)
 (done ap55)
 (done ap56)
 (done ap57)
 (done ap58)
 (done ap59)
 (done ap60)
 (done ap61)
 (done ap62)
 (done ap63)
 (done ap64)
 (done ap65)
 (done ap66)
 (done ap67)
 (done ap68)
 (done ap69)
 (done ap70)
 (done ap71)
 (done ap72)
 (done ap73)
 (done ap74)
 (done ap75)
 (done ap76)
 (done ap77)
 (done ap78)
 (done ap79)
 (done ap80)
 (done ap81)
 (done ap82)
 (done ap83)
 (done ap84)
 (done ap85)
 (done ap86)
 (done ap87)
 (done ap88)
 (done ap89)
 (done ap90)
 (done ap91)
 (done ap92)
 (done ap93)
 (done ap94)
 (done ap95)
 (done ap96)
 (done ap97)
 (done ap98)
 (done ap99)
 (done ap100)
  )) 
  )

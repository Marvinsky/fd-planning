(define (problem elevators-time-p24_18_1)
(:domain elevators-time-numeric)

(:objects 
f0 f1 f2 f3 f4 f5 f6 f7 f8 f9 f10 f11 f12 f13 f14 f15 f16 f17 f18 f19 f20 f21 f22 f23 f24  - floor
p0 p1 p2 p3 p4 p5 p6 p7 p8 p9 p10 p11 p12 p13 p14 p15 p16 p17  - passenger
fast0 fast1  - fast-elevator
slow0-0 slow1-0 slow2-0 - slow-elevator
)

(:init
(above f0 f1) (above f0 f2) (above f0 f3) (above f0 f4) (above f0 f5) (above f0 f6) (above f0 f7) (above f0 f8) (above f0 f9) (above f0 f10) (above f0 f11) (above f0 f12) (above f0 f13) (above f0 f14) (above f0 f15) (above f0 f16) (above f0 f17) (above f0 f18) (above f0 f19) (above f0 f20) (above f0 f21) (above f0 f22) (above f0 f23) (above f0 f24) 
(above f1 f2) (above f1 f3) (above f1 f4) (above f1 f5) (above f1 f6) (above f1 f7) (above f1 f8) (above f1 f9) (above f1 f10) (above f1 f11) (above f1 f12) (above f1 f13) (above f1 f14) (above f1 f15) (above f1 f16) (above f1 f17) (above f1 f18) (above f1 f19) (above f1 f20) (above f1 f21) (above f1 f22) (above f1 f23) (above f1 f24) 
(above f2 f3) (above f2 f4) (above f2 f5) (above f2 f6) (above f2 f7) (above f2 f8) (above f2 f9) (above f2 f10) (above f2 f11) (above f2 f12) (above f2 f13) (above f2 f14) (above f2 f15) (above f2 f16) (above f2 f17) (above f2 f18) (above f2 f19) (above f2 f20) (above f2 f21) (above f2 f22) (above f2 f23) (above f2 f24) 
(above f3 f4) (above f3 f5) (above f3 f6) (above f3 f7) (above f3 f8) (above f3 f9) (above f3 f10) (above f3 f11) (above f3 f12) (above f3 f13) (above f3 f14) (above f3 f15) (above f3 f16) (above f3 f17) (above f3 f18) (above f3 f19) (above f3 f20) (above f3 f21) (above f3 f22) (above f3 f23) (above f3 f24) 
(above f4 f5) (above f4 f6) (above f4 f7) (above f4 f8) (above f4 f9) (above f4 f10) (above f4 f11) (above f4 f12) (above f4 f13) (above f4 f14) (above f4 f15) (above f4 f16) (above f4 f17) (above f4 f18) (above f4 f19) (above f4 f20) (above f4 f21) (above f4 f22) (above f4 f23) (above f4 f24) 
(above f5 f6) (above f5 f7) (above f5 f8) (above f5 f9) (above f5 f10) (above f5 f11) (above f5 f12) (above f5 f13) (above f5 f14) (above f5 f15) (above f5 f16) (above f5 f17) (above f5 f18) (above f5 f19) (above f5 f20) (above f5 f21) (above f5 f22) (above f5 f23) (above f5 f24) 
(above f6 f7) (above f6 f8) (above f6 f9) (above f6 f10) (above f6 f11) (above f6 f12) (above f6 f13) (above f6 f14) (above f6 f15) (above f6 f16) (above f6 f17) (above f6 f18) (above f6 f19) (above f6 f20) (above f6 f21) (above f6 f22) (above f6 f23) (above f6 f24) 
(above f7 f8) (above f7 f9) (above f7 f10) (above f7 f11) (above f7 f12) (above f7 f13) (above f7 f14) (above f7 f15) (above f7 f16) (above f7 f17) (above f7 f18) (above f7 f19) (above f7 f20) (above f7 f21) (above f7 f22) (above f7 f23) (above f7 f24) 
(above f8 f9) (above f8 f10) (above f8 f11) (above f8 f12) (above f8 f13) (above f8 f14) (above f8 f15) (above f8 f16) (above f8 f17) (above f8 f18) (above f8 f19) (above f8 f20) (above f8 f21) (above f8 f22) (above f8 f23) (above f8 f24) 
(above f9 f10) (above f9 f11) (above f9 f12) (above f9 f13) (above f9 f14) (above f9 f15) (above f9 f16) (above f9 f17) (above f9 f18) (above f9 f19) (above f9 f20) (above f9 f21) (above f9 f22) (above f9 f23) (above f9 f24) 
(above f10 f11) (above f10 f12) (above f10 f13) (above f10 f14) (above f10 f15) (above f10 f16) (above f10 f17) (above f10 f18) (above f10 f19) (above f10 f20) (above f10 f21) (above f10 f22) (above f10 f23) (above f10 f24) 
(above f11 f12) (above f11 f13) (above f11 f14) (above f11 f15) (above f11 f16) (above f11 f17) (above f11 f18) (above f11 f19) (above f11 f20) (above f11 f21) (above f11 f22) (above f11 f23) (above f11 f24) 
(above f12 f13) (above f12 f14) (above f12 f15) (above f12 f16) (above f12 f17) (above f12 f18) (above f12 f19) (above f12 f20) (above f12 f21) (above f12 f22) (above f12 f23) (above f12 f24) 
(above f13 f14) (above f13 f15) (above f13 f16) (above f13 f17) (above f13 f18) (above f13 f19) (above f13 f20) (above f13 f21) (above f13 f22) (above f13 f23) (above f13 f24) 
(above f14 f15) (above f14 f16) (above f14 f17) (above f14 f18) (above f14 f19) (above f14 f20) (above f14 f21) (above f14 f22) (above f14 f23) (above f14 f24) 
(above f15 f16) (above f15 f17) (above f15 f18) (above f15 f19) (above f15 f20) (above f15 f21) (above f15 f22) (above f15 f23) (above f15 f24) 
(above f16 f17) (above f16 f18) (above f16 f19) (above f16 f20) (above f16 f21) (above f16 f22) (above f16 f23) (above f16 f24) 
(above f17 f18) (above f17 f19) (above f17 f20) (above f17 f21) (above f17 f22) (above f17 f23) (above f17 f24) 
(above f18 f19) (above f18 f20) (above f18 f21) (above f18 f22) (above f18 f23) (above f18 f24) 
(above f19 f20) (above f19 f21) (above f19 f22) (above f19 f23) (above f19 f24) 
(above f20 f21) (above f20 f22) (above f20 f23) (above f20 f24) 
(above f21 f22) (above f21 f23) (above f21 f24) 
(above f22 f23) (above f22 f24) 
(above f23 f24) 

(lift-at fast0 f4)
(= (passengers fast0) 0)
(= (capacity fast0) 6)
(reachable-floor fast0 f0)(reachable-floor fast0 f4)(reachable-floor fast0 f8)(reachable-floor fast0 f12)(reachable-floor fast0 f16)(reachable-floor fast0 f20)(reachable-floor fast0 f24)

(lift-at fast1 f4)
(= (passengers fast1) 0)
(= (capacity fast1) 6)
(reachable-floor fast1 f0)(reachable-floor fast1 f4)(reachable-floor fast1 f8)(reachable-floor fast1 f12)(reachable-floor fast1 f16)(reachable-floor fast1 f20)(reachable-floor fast1 f24)

(lift-at slow0-0 f8)
(= (passengers slow0-0) 0)
(= (capacity slow0-0) 4)
(reachable-floor slow0-0 f0)(reachable-floor slow0-0 f1)(reachable-floor slow0-0 f2)(reachable-floor slow0-0 f3)(reachable-floor slow0-0 f4)(reachable-floor slow0-0 f5)(reachable-floor slow0-0 f6)(reachable-floor slow0-0 f7)(reachable-floor slow0-0 f8)

(lift-at slow1-0 f13)
(= (passengers slow1-0) 0)
(= (capacity slow1-0) 4)
(reachable-floor slow1-0 f8)(reachable-floor slow1-0 f9)(reachable-floor slow1-0 f10)(reachable-floor slow1-0 f11)(reachable-floor slow1-0 f12)(reachable-floor slow1-0 f13)(reachable-floor slow1-0 f14)(reachable-floor slow1-0 f15)(reachable-floor slow1-0 f16)

(lift-at slow2-0 f21)
(= (passengers slow2-0) 0)
(= (capacity slow2-0) 4)
(reachable-floor slow2-0 f16)(reachable-floor slow2-0 f17)(reachable-floor slow2-0 f18)(reachable-floor slow2-0 f19)(reachable-floor slow2-0 f20)(reachable-floor slow2-0 f21)(reachable-floor slow2-0 f22)(reachable-floor slow2-0 f23)(reachable-floor slow2-0 f24)

(passenger-at p0 f3)
(passenger-at p1 f21)
(passenger-at p2 f7)
(passenger-at p3 f17)
(passenger-at p4 f6)
(passenger-at p5 f0)
(passenger-at p6 f16)
(passenger-at p7 f20)
(passenger-at p8 f19)
(passenger-at p9 f18)
(passenger-at p10 f22)
(passenger-at p11 f14)
(passenger-at p12 f12)
(passenger-at p13 f7)
(passenger-at p14 f8)
(passenger-at p15 f16)
(passenger-at p16 f8)
(passenger-at p17 f9)

(= (travel-slow f0 f1) 12) (= (travel-slow f0 f2) 20) (= (travel-slow f0 f3) 28) (= (travel-slow f0 f4) 36) (= (travel-slow f0 f5) 44) (= (travel-slow f0 f6) 52) (= (travel-slow f0 f7) 60) (= (travel-slow f0 f8) 68) (= (travel-slow f1 f2) 12) (= (travel-slow f1 f3) 20) (= (travel-slow f1 f4) 28) (= (travel-slow f1 f5) 36) (= (travel-slow f1 f6) 44) (= (travel-slow f1 f7) 52) (= (travel-slow f1 f8) 60) (= (travel-slow f2 f3) 12) (= (travel-slow f2 f4) 20) (= (travel-slow f2 f5) 28) (= (travel-slow f2 f6) 36) (= (travel-slow f2 f7) 44) (= (travel-slow f2 f8) 52) (= (travel-slow f3 f4) 12) (= (travel-slow f3 f5) 20) (= (travel-slow f3 f6) 28) (= (travel-slow f3 f7) 36) (= (travel-slow f3 f8) 44) (= (travel-slow f4 f5) 12) (= (travel-slow f4 f6) 20) (= (travel-slow f4 f7) 28) (= (travel-slow f4 f8) 36) (= (travel-slow f5 f6) 12) (= (travel-slow f5 f7) 20) (= (travel-slow f5 f8) 28) (= (travel-slow f6 f7) 12) (= (travel-slow f6 f8) 20) (= (travel-slow f7 f8) 12) 

(= (travel-slow f8 f9) 12) (= (travel-slow f8 f10) 20) (= (travel-slow f8 f11) 28) (= (travel-slow f8 f12) 36) (= (travel-slow f8 f13) 44) (= (travel-slow f8 f14) 52) (= (travel-slow f8 f15) 60) (= (travel-slow f8 f16) 68) (= (travel-slow f9 f10) 12) (= (travel-slow f9 f11) 20) (= (travel-slow f9 f12) 28) (= (travel-slow f9 f13) 36) (= (travel-slow f9 f14) 44) (= (travel-slow f9 f15) 52) (= (travel-slow f9 f16) 60) (= (travel-slow f10 f11) 12) (= (travel-slow f10 f12) 20) (= (travel-slow f10 f13) 28) (= (travel-slow f10 f14) 36) (= (travel-slow f10 f15) 44) (= (travel-slow f10 f16) 52) (= (travel-slow f11 f12) 12) (= (travel-slow f11 f13) 20) (= (travel-slow f11 f14) 28) (= (travel-slow f11 f15) 36) (= (travel-slow f11 f16) 44) (= (travel-slow f12 f13) 12) (= (travel-slow f12 f14) 20) (= (travel-slow f12 f15) 28) (= (travel-slow f12 f16) 36) (= (travel-slow f13 f14) 12) (= (travel-slow f13 f15) 20) (= (travel-slow f13 f16) 28) (= (travel-slow f14 f15) 12) (= (travel-slow f14 f16) 20) (= (travel-slow f15 f16) 12) 

(= (travel-slow f16 f17) 12) (= (travel-slow f16 f18) 20) (= (travel-slow f16 f19) 28) (= (travel-slow f16 f20) 36) (= (travel-slow f16 f21) 44) (= (travel-slow f16 f22) 52) (= (travel-slow f16 f23) 60) (= (travel-slow f16 f24) 68) (= (travel-slow f17 f18) 12) (= (travel-slow f17 f19) 20) (= (travel-slow f17 f20) 28) (= (travel-slow f17 f21) 36) (= (travel-slow f17 f22) 44) (= (travel-slow f17 f23) 52) (= (travel-slow f17 f24) 60) (= (travel-slow f18 f19) 12) (= (travel-slow f18 f20) 20) (= (travel-slow f18 f21) 28) (= (travel-slow f18 f22) 36) (= (travel-slow f18 f23) 44) (= (travel-slow f18 f24) 52) (= (travel-slow f19 f20) 12) (= (travel-slow f19 f21) 20) (= (travel-slow f19 f22) 28) (= (travel-slow f19 f23) 36) (= (travel-slow f19 f24) 44) (= (travel-slow f20 f21) 12) (= (travel-slow f20 f22) 20) (= (travel-slow f20 f23) 28) (= (travel-slow f20 f24) 36) (= (travel-slow f21 f22) 12) (= (travel-slow f21 f23) 20) (= (travel-slow f21 f24) 28) (= (travel-slow f22 f23) 12) (= (travel-slow f22 f24) 20) (= (travel-slow f23 f24) 12) 


(= (travel-fast f0 f4) 13) (= (travel-fast f0 f8) 17) (= (travel-fast f0 f12) 20) (= (travel-fast f0 f16) 22) (= (travel-fast f0 f20) 24) (= (travel-fast f0 f24) 26) 

(= (travel-fast f4 f8) 13) (= (travel-fast f4 f12) 17) (= (travel-fast f4 f16) 20) (= (travel-fast f4 f20) 22) (= (travel-fast f4 f24) 24) 

(= (travel-fast f8 f12) 13) (= (travel-fast f8 f16) 17) (= (travel-fast f8 f20) 20) (= (travel-fast f8 f24) 22) 

(= (travel-fast f12 f16) 13) (= (travel-fast f12 f20) 17) (= (travel-fast f12 f24) 20) 

(= (travel-fast f16 f20) 13) (= (travel-fast f16 f24) 17) 

(= (travel-fast f20 f24) 13) 

)

(:goal
(and
(passenger-at p0 f14)
(passenger-at p1 f8)
(passenger-at p2 f17)
(passenger-at p3 f9)
(passenger-at p4 f21)
(passenger-at p5 f5)
(passenger-at p6 f22)
(passenger-at p7 f21)
(passenger-at p8 f22)
(passenger-at p9 f0)
(passenger-at p10 f0)
(passenger-at p11 f22)
(passenger-at p12 f9)
(passenger-at p13 f9)
(passenger-at p14 f0)
(passenger-at p15 f2)
(passenger-at p16 f4)
(passenger-at p17 f21)
))

(:metric minimize (total-time))

)

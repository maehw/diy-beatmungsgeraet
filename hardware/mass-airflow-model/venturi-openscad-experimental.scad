// this file by maehw/diy-beatmungsgeraet is licensed under the
// GNU General Public License v3.0

// disclaimer: this design is very experimental!

$fn = 25; // increase number to improve result (and to slow down rendering)

D = 13;
Li = 14; // Li >= D; length of the inlet
d = 8.5;
a1 = 21; // 21 +-1 degree
a2 = 11; // 7..15 degree
z = D/4;
x = d/2;

p = 1; // pipe system diameter
t = 2; // main flow tube system thickness (take care to include pipe diameters!)

// calculate part heights (lengths)
Lc = (D-d)/(2*tan(a1/2)); // calculate adjacent side
Lt = D; // +- 0.05*d
Ld = (D-d)/(2*tan(a2/2));
L = Li+Lc+Lt+Ld;

ypi = Li-z; // y coordinate of pipe inlet connection (towards sensor)
ypt = Li+Lc+x; // y coordinate of pipe throat connection (towards sensor)
xp = D/2 + 0.5 + p; // radius from inner of main tube to where the small pipes go towards the sensor
Lp = ypt - ypi; // length between the two pipe connections
Ls = 5; // sensor connection spacing
Lps = 4; // length of parallel horizontal pipes towards sensor

difference()
{
    difference()
    {
        // solid "outer" body
        union()
        {
            cylinder(L, d/2 + 0.5 + p + t, d/2 + 0.5 + p + t);

            translate([0,0,ypi-2*p]) cylinder(Lp+4*p, D/2 + 8, D/2 + 8);
        }

        union()
        {
            // main flow tube system
            union()
            {
                // cylindrical inlet
                cylinder(Li, D/2, D/2);

                // convergent entrance
                translate([0, 0, Li]) cylinder(Lc, D/2, d/2);

                // throat
                translate([0, 0, Li+Lc]) cylinder(Lt, d/2, d/2);

                // divergent outlet
                translate([0, 0, Li+Lc+Lt]) cylinder(Ld, d/2, D/2);
            }
            
            // add pipe system
            union()
            {
                // horizontal pipe connection in the inlet
                translate([0,0,ypi]) rotate([0,90,0]) cylinder(xp, p, p);
                // horizontal pipe connection in the throat
                translate([0,0,ypt]) rotate([0,90,0]) cylinder(xp, p, p);

                // make two smooth joints at the edges
                translate([xp,0,ypi]) sphere(p);
                translate([xp,0,ypt]) sphere(p);
                
                // horizontal lower pipe connection (from inlet) to the sensor
                translate([xp,0,ypi+(Lp-Ls)/2]) rotate([0,90,0]) cylinder(Lps, p, p);
                // horizontal upper pipe connection (from throat) to the sensor
                translate([xp,0,ypt-(Lp-Ls)/2]) rotate([0,90,0]) cylinder(Lps, p, p);

                // make two smooth joints at the edges
                translate([xp,0,ypi+(Lp-Ls)/2]) sphere(p);
                translate([xp,0,ypt-(Lp-Ls)/2]) sphere(p);
                
                // vertical pipe connections towards the sensor
                translate([xp,0,ypi]) cylinder((Lp-Ls)/2, p, p);
                translate([xp,0,ypt - (Lp-Ls)/2]) cylinder((Lp-Ls)/2, p, p);
                
                // horizontal lower pipe connection (from inlet) to the sensor
                translate([xp+Lps,0,ypi+(Lp-Ls)/2]) rotate([0,90,0]) cylinder(3, p, p+0.5);
                // horizontal upper pipe connection (from throat) to the sensor
                translate([xp+Lps,0,ypt-(Lp-Ls)/2]) rotate([0,90,0]) cylinder(3, p, p+0.5);
            }
        }
    }
    
    // subtract to have a look inside
    translate([-250,0,-1]) cube([500, 500, L+2]);
}
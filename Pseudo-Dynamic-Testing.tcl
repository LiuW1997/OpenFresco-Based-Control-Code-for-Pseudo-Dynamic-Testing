
wipe

# ------------------------------
# Start of model generation
# ------------------------------
logFile "TestColumn_master.log"
# create ModelBuilder (with two-dimensions and 2 DOF/node)
model BasicBuilder -ndm 2 -ndf 3

# Load OpenFresco package
# -----------------------
# (make sure all dlls are in the same folder as openSees.exe)
loadPackage OpenFresco

# Define geometry for model
# -------------------------
# node $tag $xCrd $yCrd $mass

# 四节点--------------------------------------------------------------------------------------------------------
node  1     0.0    0.0
# node  2     0.0    2.0  -mass 32653 32653 0.0
# node  2     0.0    2.0  -mass 40816 40816 0.0
# node  2     0.0    2.0  -mass 1828 1828 0.0
# node  2     0.0    2.0  -mass 42644 42644 0.0
node  2     0.0    2.0  -mass 34481 34481 0.0
node  3     -1.0    2.0
node  4     -1.0    0.0  
# -mass 43794.4 43794.4 0.0
# -mass 894 894 0.0
# 四节点--------------------------------------------------------------------------------------------------------

# set the boundary conditions
# fix $tag $DX $DY $RZ
fix 1   1  1  1
fix 2   0  0  0
fix 3   0  0  0
fix 4   1  1  1

#-----------------------------------------------------------------------------------------------------

#-----------------------------------------------------------------------------------------------------
set Ramptime 0.5
#-----------------------------------------------------------------------------------------------------

# Define control points
# ---------------------
# expControlPoint $cpTag <-node $nodeTag> $dof $rspType <-fact $f> <-lim $l $u> <-isRel> ...
expControlPoint 1  1 disp
expControlPoint 2  1 disp  1 force 

# Define experimental control
# ---------------------------
# expControl SimFEAdapter $tag ipAddr $ipPort -trialCP $cpTags -outCP $cpTags
# expControl SimFEAdapter 1 "127.0.0.1" 44000 -trialCP 1 -outCP 2
expControl  MTSCsi   1  "C:/opensees_projects/OSN Models/task_1_D_QS/x.mtscs"  $Ramptime  -trialCP  1  -outCP  2 

# Define experimental setup
# -------------------------
# expSetup OneActuator $tag <-control $ctrlTag> $dir -sizeTrialOut $t $o <-trialDispFact $f> ...
expSetup OneActuator 1 -control 1 2 -sizeTrialOut 3 3 -ctrlDispFact -1000 -daqDispFact -1.0 -daqForceFact -1000

# Define experimental site
# ------------------------
# expSite LocalSite $tag $setupTag
expSite LocalSite 1 1

# Define coordinate transformation
# --------------------------------
# geomTransf Linear $transfTag
# geomTransf PDelta 1
geomTransf Linear 1
geomTransf Linear 2
geomTransf Linear 3

# Define experimental element
# ---------------------------
# expElement beamColumn $eleTag $iNode $jNode $transTag 杝ite $siteTag 杋nitStif $Kij ... <杋Mod> <杛ho $rho>
# l = 0.2m
# expElement beamColumn 1 1 2 1 -site 1 -initStif 2.6e10 0 0 0 1.04e11 -1.04e10 0 -1.04e10 1.39e9
# l = 2m
expElement beamColumn 1 1 2 1 -site 1 -initStif 2.6e9 0 0 0 1.04e8 -1.04e8 0 -1.04e8 1.39e8
# expElement beamColumn 1 1 2 1 -site 1 -initStif 2.9e9 0 0 0 1.15e8 -1.15e8 0 -1.15e8 1.5e8


element elasticBeamColumn 2 2 3 0.16e0 3.25e0 8e0 2


element elasticBeamColumn 3 3 4 0.16e0 3.25e0 8e0 3


#-----------------------------------------------------------------------------------------------------

loadConst -time 0.0

set time_gravity 1
timeSeries Linear $time_gravity  ;

set pattern_gravity 1

set P [expr 9.81 * 34481];
# set P1 [expr 9.81*894];
pattern Plain $pattern_gravity $time_gravity {
        # Create nodal loads at nodes 2
        #    nd    FX          FY  MZ 
        load  1   0.0  0.0  0.0
		load  2   0.0  [expr -$P]  0.0
        # load  3   0.0  [expr -$P1]  0.0
		# load  3   0.0  [expr -$P1] 0.0
}

constraints Plain  ;
numberer Plain  ;
system BandGen  ;

set Max_check 10  ;
test NormDispIncr 1.0e-7 $Max_check 0  ;

algorithm KrylovNewton  ;

integrator LoadControl 0.1  ;

analysis Static

# 开始重力分析
set mass_beam [expr 34481]
set data [list]
for {set j 0} {$j < $Max_check} {incr j} {
    set ok [analyze 1]
    set u_disp [nodeDisp 2 2]
    set loadFactor [getLoadFactor 1]
    lappend data [list $u_disp [expr $loadFactor * $mass_beam]]
}

if {$ok == 0} {
    puts "Gravity analysis succeed."
}

# 将数据保存到文件
set outFile [open "./result/deformation_gravity1.txt" "w"]
foreach row $data {
    puts $outFile "[join $row \t]"
}
close $outFile

#-----------------------------------------------------------------------------------------------------

# ------------------------------
# Start of recorder generation
# ------------------------------
# create the recorder objects
recorder Node -file ./result/Master_Node_Dsp.out -time -node 2 -dof 1 2 3 disp
recorder Node -file ./result/Master_Node_Vel.out -time -node 2 -dof 1 2 3 vel
recorder Node -file ./result/Master_Node_Acc.out -time -node 2 -dof 1 2 3 accel

recorder Element -file ./result/Master_Elmt_Frc1.out     -time -ele 1 forces
recorder Element -file ./result/Master_Elmt_Frc2.out     -time -ele 2 forces
recorder Element -file ./result/Master_Elmt_ctrlDsp.out -time -ele 1 ctrlDisp
recorder Element -file ./result/Master_Elmt_daqDsp.out  -time -ele 1 daqDisp

# 单元力
recorder Element -file "./result/bear1_Force.out" -time -ele 1 globalForce

# 基础反力
recorder Node -file "./result/nodeforce1.out" -time -node 1 -dof 1 2 3 reaction
recorder Node -file "./result/nodeforce11.out" -time -node 2 -dof 1 2 3 reaction
recorder Node -file "./result/nodeDisp1.out" -time -node 1 -dof 1 2 3 disp
recorder Node -file "./result/nodeDisp11.out" -time -node 2 -dof 1 2 3 disp
recorder Node -file "./result/nodeVel11.out" -time -node 2 -dof 1 2 3 vel
recorder Node -file "./result/nodeAccel11.out" -time -node 2 -dof 1 2 3 accel

# --------------------------------
# End of recorder generation
# --------------------------------

#-----------------------------------------------------------------------------------------------------

loadConst -time 0.0

# 
set alphaM 2.0272108104392035e-5
set betaKinit 6.806016777169761e-5
set betaK 0
set betaKcomm 0
rayleigh $alphaM $betaK $betaKinit $betaKcomm

set GM_file "RSN4098_PARK2004_C01090_0.005_1.33.txt"

set dt [expr 0.005 * 0.4472]

set Excitation {}
set Excitation_total 0

set fid [open $GM_file r]
while {![eof $fid]} {
    gets $fid line
    if {[string trim $line] eq ""} {
        continue
    }
    lappend Excitation $line
    incr Excitation_total
}
close $fid

puts "Total number of excitation points: $Excitation_total"


set accl1 [list]
for {set j 0} {$j < $Excitation_total} {incr j} {
    lappend accl1 [list [expr $j * $dt] [lindex $Excitation $j]]
}


set fid_out [open "./result/accl1_output.txt" w]
foreach item $accl1 {
    set time_step [lindex $item 0]
    set acceleration [lindex $item 1]
    puts $fid_out "$time_step $acceleration"
}
close $fid_out

set time_acc 2


set GM_factor 3.4

timeSeries Path $time_acc -dt $dt -filePath $GM_file -factor $GM_factor

set pattern_GM 2
set direction_GM 1   

pattern UniformExcitation $pattern_GM $direction_GM -accel $time_acc



# ------------------------------
# Start of analysis generation
# ------------------------------
# create the system of equations
system BandGeneral
# create the DOF numberer
numberer Plain
# create the constraint handler

constraints Transformation
# create the convergence test
#test NormDispIncr 1.0e-8 25
#test NormUnbalance 1.0e-8 25
# test EnergyIncr 1.0e-7 10000
test NormDispIncr 1.0e-7 50000
# create the integration scheme-integrator Newmark 0.5 0.25

algorithm KrylovNewton
# algorithm KrylovNewton 
#algorithm Linear -initial
# create the analysis object 
set gama 0.5
set beta 0.25
integrator Newmark $gama $beta
analysis Transient
# ------------------------------
# End of analysis generation
# ------------------------------


# ------------------------------
# Finally perform the analysis
# ------------------------------
record

# open output file for writing
set outFileID [open elapsedTime.txt w]
# perform the transient analysis
set tTot [time {
    for {set i 1} {$i < 2100} {incr i} {
        set t [time {analyze  1  [expr $dt/1]}]
        puts $outFileID $t
		# puts $i
        puts "step $i"
    }
}]
puts "\nElapsed Time = $tTot \n"
# close the output file
close $outFileID
# wipeExp
# wipe
exit
# --------------------------------
# End of analysis
# --------------------------------

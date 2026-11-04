set timing_file [file join [pwd] impl pnr ${build_name}_tr_content.html]
set f [open $timing_file r]
set html [read $f]
close $f
foreach kind {Setup Hold} {
 set pat [format {Numbers of %s Violated Endpoints</td>\s*<td[^>]*>([0-9]+)</td>} $kind]
 if {![regexp $pat $html all n] || $n!=0} {
  if {[file exists [file join [pwd] impl pnr ${build_name}.fs]]} {
   file rename -force [file join [pwd] impl pnr ${build_name}.fs] [file join [pwd] impl pnr ${build_name}.fs.DO_NOT_PROGRAM]
  }
  error "Timing failed: $kind"
 }
}
puts "PASS: $build_name setup/hold violations zero"

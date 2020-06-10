<?php

$dllfile = "ExtremePhysics.dll";
$dllinit = "extremephysics_init";

$output_newline = "\r\n";

$data = file_get_contents("dllfunctions.txt");
$lines = explode("\n",str_replace("\r","",$data));
$output_init = '#define '.$dllinit.$output_newline.'var dllfile;'.$output_newline.'if is_string(argument0) then dllfile = argument0;'.$output_newline.'else dllfile = \''.$dllfile.'\';'.$output_newline.$output_newline;
$output_scripts = '';

$kind = 'dll_stdcall';

foreach($lines as $l=>$line) {
	$words = explode(' ',$line);
	if(count($words)!=2) continue;
	if($words[0]=='kind') {
		if($words[1]=='dll-stdcall') $kind = 'dll_stdcall';
		if($words[1]=='dll-cdecl') $kind = 'dll_cdecl';
	}
	if($words[0]=='function') {
		if(preg_match('/^(.+)\((.*)\):(.+)$/U',$words[1],$match)) {

			$function_name = $match[1];
			$function_argumentcount = 0;
			$function_argumentnames = '';
			$function_argumenttypes = '';
			$function_arguments = '';
			$function_restype = 'ty_'.$match[3];

			if($match[2]!='') {
				$a = explode(';',$match[2]);
				foreach($a as $i=>$b) {
					$c = explode(':',$b);
					if(count($c)!=2) {
						die('Error in argument '.($i+1).' on line '.($l+1).'.'."\r\n");
					}
					if($function_argumentnames!='') $function_argumentnames .= ',';
					$function_argumentnames .= $c[0];
					if($function_argumenttypes!='') $function_argumenttypes .= ',';
					$function_argumenttypes .= 'ty_'.$c[1];
					if($function_arguments!='') $function_arguments .= ',';
					$function_arguments .= 'argument'.$function_argumentcount;
					$function_argumentcount++;
				}
			}

			$output_init .= 'global.define_'.$function_name.' = external_define(dllfile,\''.$function_name.'\','.$kind.','.$function_restype.','.$function_argumentcount.(($function_argumentcount>0)?(','.$function_argumenttypes):('')).');'.$output_newline;
			$output_scripts .= '#define '.$function_name.$output_newline.'// '.$match[1].'('.$function_argumentnames.')'.$output_newline.'external_call(global.define_'.$function_name.(($function_arguments!='')?(','.$function_arguments):('')).');'.$output_newline;

		}
	}
}

$output = $output_init."\n".$output_scripts;
file_put_contents("ExtremePhysics.gml",$output);

?>
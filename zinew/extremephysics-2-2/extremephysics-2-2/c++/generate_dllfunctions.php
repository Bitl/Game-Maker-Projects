<?php

$files = explode("\n",str_replace("\r","",file_get_contents('dllfiles.txt')));

$output = 'kind dll-cdecl'."\r\n"."\r\n";

foreach($files as $file) {
if($file!='') {
echo $file."\n";

$data = str_replace("\r","",file_get_contents($file));

$data = preg_replace('/\/\*(.*)\*\//sU','',$data);

$count = preg_match_all('/gmexport ([^ ]+) ([^ ]+)\(([^)]*)\) \{/',$data,$matches);
for($i=0;$i<$count;$i++) {

$output .= 'function '.$matches[2][$i].'(';
$count2 = preg_match_all('/([^ ,\n\t]+) ([^ ,\n\t]+)/',$matches[3][$i],$matches2);
for($i2=0;$i2<$count2;$i2++) {
if($i2>0) $output .= ';';
$output .= $matches2[2][$i2].':'.(($matches2[1][$i2]=='double')?('real'):('string'));
}
$output .= '):'.(($matches[1][$i]=='double')?('real'):('string'))."\r\n";

}

$output .= "\r\n";

}
}

file_put_contents('dllfunctions.txt',$output);

?>
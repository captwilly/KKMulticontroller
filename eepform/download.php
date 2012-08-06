<?php

require_once 'config.inc';

/* 
 * Check input data
 */

if (!isset($_POST['Version'])) {
	die('Error: Required parameter "Version" is not set');
}

$Version = (int)$_POST['Version'];
if ($Version !== 1) {
	die('Error: Unsupported EEPROM version ' . $_POST['Version']);
}

if (!isset($_POST['RollGyroDirection'])) {
	die('Error: Required parameter "RollGyroDirection" is not set');
}

$RollGyroDirection = (int)$_POST['RollGyroDirection'];
if ($RollGyroDirection !== 0 && $RollGyroDirection !== 1) {
	die('Error: Required parameter "RollGyroDirection" is
			 not in supported range - ' . $RollGyroDirection);
}

if (!isset($_POST['PitchGyroDirection'])) {
	die('Error: Required parameter "PitchGyroDirection" is not set');
}

$PitchGyroDirection = (int)$_POST['PitchGyroDirection'];
if ($PitchGyroDirection !== 0 && $PitchGyroDirection !== 1) {
	die('Error: Required parameter "PitchGyroDirection" is
			 not in supported range - ' . $PitchGyroDirection);
}

if (!isset($_POST['YawGyroDirection'])) {
	die('Error: Required parameter "YawGyroDirection" is not set');
}

$YawGyroDirection = (int)$_POST['YawGyroDirection'];
if ($YawGyroDirection !== 0 && $YawGyroDirection !== 1) {
	die('Error: Required parameter "YawGyroDirection" is
			 not in supported range - ' . $YawGyroDirection);
}

if (!isset($_POST['RxRollZero'])) {
	die('Error: Required parameter "RxRollZero" is not set');
}

$RxRollZero = (int)$_POST['RxRollZero'];
if ($RxRollZero > MAX_ZERO_OFFSET || $RxRollZero < MIN_ZERO_OFFSET) {
	die('Error: Required parameter "RxRollZero" is
			 not in supported range - ' . $RxRollZero);
}

if (!isset($_POST['RxPitchZero'])) {
	die('Error: Required parameter "RxPitchZero" is not set');
}

$RxPitchZero = (int)$_POST['RxPitchZero'];
if ($RxPitchZero > MAX_ZERO_OFFSET || $RxPitchZero < MIN_ZERO_OFFSET) {
	die('Error: Required parameter "RxPitchZero" is
			 not in supported range - ' . $RxPitchZero);
}

if (!isset($_POST['RxYawZero'])) {
	die('Error: Required parameter "RxYawZero" is not set');
}

$RxYawZero = (int)$_POST['RxYawZero'];
if ($RxYawZero > MAX_ZERO_OFFSET || $RxYawZero < MIN_ZERO_OFFSET) {
	die('Error: Required parameter "RxYawZero" is
			 not in supported range - ' . $RxYawZero);
}

if (!isset($_POST['Comment'])) {
	die('Error: Required parameter "Comment" is not set');
}

$Comment = $_POST['Comment'];
if (strlen($Comment) > 99) {
	die('Error: Required parameter "Comment" is
			 not in supported range - ' . htmlspecialchars($Comment));
}


/* 
 * Form ASCII-HEX string
 */
$result = '';

// CRC Magic
$result .= EEPROM_SETTINGS_MAGIC_LE;

// Version
$result .= sprintf('%02X', $Version);

// Gyro reversing
$result .= sprintf('%02X', $RollGyroDirection);
$result .= sprintf('%02X', $PitchGyroDirection);
$result .= sprintf('%02X', $YawGyroDirection);

// Zero offset
$result .= sprintf('%02X%02X', $RxRollZero & 0xff, ($RxRollZero >> 8) & 0xff);
$result .= sprintf('%02X%02X', $RxPitchZero & 0xff, ($RxPitchZero >> 8) & 0xff);
$result .= sprintf('%02X%02X', $RxYawZero & 0xff, ($RxYawZero >> 8) & 0xff);

// Comment
for ($i = 0; $i < strlen($Comment); $i++) {
	$result .= sprintf('%02X', ord($Comment[$i]));
}
$result .= '00'; // Null-terminated string

$ihex = '';
$lines = str_split($result, BYTES_PER_IHEX_LINE * 2);
foreach ($lines as $line_num => $line) {
	$sum = 0;
	$address = $line_num * BYTES_PER_IHEX_LINE;
	// Bytes in line
	$ihex .= ':' . sprintf('%02X', strlen($line) / 2);
	$sum += strlen($line) / 2;
	// Block address (big-endian)
	$ihex .= sprintf('%02X%02X', ($address >> 8) & 0xff, $address & 0xff);
	$sum += (($address >> 8) & 0xff) + ($address & 0xff);
	// Data-type - "data"
	$ihex .= '00';
	$ihex .= $line . '';
	
	
	$bytes = str_split($line, 2);
	foreach ($bytes as $byte) {
		$sum += (int)hexdec($byte);		
	}
	$sum = (0 - $sum) & 0xff;
	$ihex .= sprintf("%02X\r\n", $sum);
}
// EOF
$ihex .= ":00000001FF\r\n";

header("Content-Type: application/force-download");
//set the value of the fields in Opened dailog box
header('Content-Disposition: attachment; filename="eeprom.hex"');

echo($ihex);
?>
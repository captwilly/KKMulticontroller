<?php

require_once 'config.inc';

/**
 * Handle file uploads via XMLHttpRequest
 */
class qqUploadedFileXhr {
    /**
     * Save the file to the specified path
     * @return boolean TRUE on success
     */
    function save($path) {    
        $input = fopen("php://input", "r");
        $temp = tmpfile();
        $realSize = stream_copy_to_stream($input, $temp);
        fclose($input);
        
        if ($realSize != $this->getSize()){            
            return false;
        }
        
        $target = fopen($path, "w");        
        fseek($temp, 0, SEEK_SET);
        stream_copy_to_stream($temp, $target);
        fclose($target);
        
        return true;
    }
    function read() {
    	return file('php://input');
    }
    function getName() {
        return $_GET['qqfile'];
    }
    function getSize() {
        if (isset($_SERVER["CONTENT_LENGTH"])){
            return (int)$_SERVER["CONTENT_LENGTH"];            
        } else {
            throw new Exception('Getting content length is not supported.');
        }      
    }   
}

/**
 * Handle file uploads via regular form post (uses the $_FILES array)
 */
class qqUploadedFileForm {  
    /**
     * Save the file to the specified path
     * @return boolean TRUE on success
     */
    function save($path) {
        if(!move_uploaded_file($_FILES['qqfile']['tmp_name'], $path)){
            return false;
        }
        return true;
    }
    function read() {
    	return file($_FILES['qqfile']['tmp_name']);
    }
    function getName() {
        return $_FILES['qqfile']['name'];
    }
    function getSize() {
        return $_FILES['qqfile']['size'];
    }
}

class qqFileUploader {
    private $allowedExtensions = array();
    private $sizeLimit = 10485760;
    private $file;

    function __construct(array $allowedExtensions = array(), $sizeLimit = 10485760){        
        $allowedExtensions = array_map("strtolower", $allowedExtensions);
            
        $this->allowedExtensions = $allowedExtensions;        
        $this->sizeLimit = $sizeLimit;
        
        $this->checkServerSettings();       

        if (isset($_GET['qqfile'])) {
            $this->file = new qqUploadedFileXhr();
        } elseif (isset($_FILES['qqfile'])) {
            $this->file = new qqUploadedFileForm();
        } else {
            $this->file = false; 
        }
    }
    
    private function checkServerSettings(){        
        $postSize = $this->toBytes(ini_get('post_max_size'));
        $uploadSize = $this->toBytes(ini_get('upload_max_filesize'));        
        
        if ($postSize < $this->sizeLimit || $uploadSize < $this->sizeLimit){
            $size = max(1, $this->sizeLimit / 1024 / 1024) . 'M';             
            die("{'error':'increase post_max_size and upload_max_filesize to $size'}");    
        }        
    }
    
    private function toBytes($str){
        $val = trim($str);
        $last = strtolower($str[strlen($str)-1]);
        switch($last) {
            case 'g': $val *= 1024;
            case 'm': $val *= 1024;
            case 'k': $val *= 1024;        
        }
        return $val;
    }
    
    /**
     * Returns array('success'=>true) or array('error'=>'error message')
     */
    function handleUpload(){        
        if (!$this->file){
            return array('error' => 'No files were uploaded.');
        }
        
        $size = $this->file->getSize();
        
        if ($size == 0) {
            return array('error' => 'File is empty');
        }
        
        if ($size > $this->sizeLimit) {
            return array('error' => 'File is too large');
        }
        
        $pathinfo = pathinfo($this->file->getName());
        $filename = $pathinfo['filename'];
        //$filename = md5(uniqid());
        $ext = $pathinfo['extension'];

        if($this->allowedExtensions && !in_array(strtolower($ext), $this->allowedExtensions)){
            $these = implode(', ', $this->allowedExtensions);
            return array('error' => 'File has an invalid extension, it should be one of '. $these . '.');
        }
        
        $lines = $this->file->read();
        if (FALSE === $lines) {
        	return array('error' => 'Can\'t open file');
        }

        
        $result = '';
        $eof = FALSE;
        foreach ($lines as $line_num => $line) {
        	// Remove ending newline
        	$line = rtrim($line);
        	
        	if ($line[0] != ':') {
        		return array('error' => 'Wrong format: no semicolon at beginning of line ' . $line_num); 
        	}
        	
        	if($line == ':00000001FF') {
        		if ($line_num + 1 != count($lines)) {
	        		return array('error' => 'Wrong format: EOF marker not in the end of file '); 
        		}
        		$eof = TRUE;
        		break;
        	}
        	$sum = 0;
        	
        	$line_len = (int)hexdec(substr($line, 1, 2));
        	if((strlen($line) - 11) != ($line_len * 2)) {
        		return array('error' => 'Wrong format: line size differes from reported at ' . $line_num); 
        	}
        	$sum += $line_len;
        	
        	$start_addr = (int)hexdec(substr($line, 3, 4));
        	if ($start_addr != strlen($result) / 2) {
        		return array('error' => "Gaps or non-zero offsets on non-consecutive chenks are not supported (start addr $start_addr, already have ". strlen($result) / 2 .")"); 
        	}
			$sum += ($start_addr & 0xff) + (($start_addr >> 8) & 0xff);
        	
        	$line_type = (int)hexdec(substr($line, 7, 2));
        	if (0 != $line_type) {
        		return array('error' => 'Wrong format: unsupported record type - ' . $line_type);
        	}
        	$sum += $line_type;
        	
        	$data = substr($line, 9, $line_len * 2);
        	$result .= $data;
        	
        	$bytes = str_split($data, 2);
        	foreach ($bytes as $byte) {
        		$sum += (int)hexdec($byte);
        	}
        	$sum = (0 - $sum) & 0xff;

        	$sum_stored = (int)hexdec(substr($line, strlen($line) - 2, 2));
        	if ($sum != $sum_stored) {
        		return array('error' => 'Wrong format: check sum is ' . dechex($sum) . ' but reported ' . dechex($sum_stored) . ' at line ' . $line_num . ' ' . $line);
        	}
        }
        	
        if (!$eof) {
        	return array('error' => 'Wrong format: no EOF detected');
        }
        
        $pos = 0;
        
        // Skip 4 bytes for CRC
        $pos += 8;
        
        $Version = (int)hexdec(substr($result, $pos, 2));
        $pos += 2;
        if ($Version != 1) {
        	return array('error' => 'Only EEPROM version 1 is supported. Supplies version is detected as ' . $Version);
        }
        
        $RollGyroDirection = (int)hexdec(substr($result, $pos, 2));
        $pos += 2;
        if ($RollGyroDirection != 0 && $RollGyroDirection != 1) {
    		return array('error' => 'Error: Required parameter "RollGyroDirection" is
        				not in supported range - ' . $RollGyroDirection);
        }
        
        $PitchGyroDirection = (int)hexdec(substr($result, $pos, 2));
        $pos += 2;
        if ($PitchGyroDirection != 0 && $PitchGyroDirection != 1) {
    		return array('error' => 'Error: Required parameter "PitchGyroDirection" is
        				not in supported range - ' . $PitchGyroDirection);
        }
        
        $YawGyroDirection = (int)hexdec(substr($result, $pos, 2));
        $pos += 2;
        if ($YawGyroDirection != 0 && $YawGyroDirection != 1) {
    		return array('error' => 'Error: Required parameter "YawGyroDirection" is
        				not in supported range - ' . $YawGyroDirection);
        }
        
        // Extract 16-bit little endian value
        $RxRollZero = (int)hexdec(substr($result, $pos, 2));
        $pos += 2;
        $RxRollZero += (int)hexdec(substr($result, $pos, 2)) << 8;
        $pos += 2;
        // Handle sign
        if ($RxRollZero & (1 << 15)) {
        	$RxRollZero -= 1 << 16;
        }
        if ($RxRollZero > MAX_ZERO_OFFSET || $RxRollZero < MIN_ZERO_OFFSET) {
        	return array('error' => 'Error: Required parameter "RxRollZero" is
        			not in supported range - ' . $RxRollZero);
        }
        
        $RxPitchZero = (int)hexdec(substr($result, $pos, 2));
        $pos += 2;
        $RxPitchZero += (int)hexdec(substr($result, $pos, 2)) << 8;
        $pos += 2;
        if ($RxPitchZero & (1 << 15)) {
        	$RxPitchZero -= 1 << 16;
        }
        if ($RxPitchZero > MAX_ZERO_OFFSET || $RxPitchZero < MIN_ZERO_OFFSET) {
        	return array('error' => 'Error: Required parameter "RxPitchZero" is
        			not in supported range - ' . $RxPitchZero);
        }
        
        $RxYawZero = (int)hexdec(substr($result, $pos, 2));
        $pos += 2;
        $RxYawZero += (int)hexdec(substr($result, $pos, 2)) << 8;
        $pos += 2;
        if ($RxYawZero & (1 << 15)) {
        	$RxYawZero -= 1 << 16;
        }
        if ($RxYawZero > MAX_ZERO_OFFSET || $RxYawZero < MIN_ZERO_OFFSET) {
        	return array('error' => 'Error: Required parameter "RxYawZero" is
        			not in supported range - ' . $RxYawZero);
        }
        
        $Comment = '';
        for($i = $pos; $i <= $pos + MAX_COMMENT_LENGTH * 2; $i += 2) {
        	$char = (int)hexdec(substr($result, $i, 2));
        	if ($char == 0) {
        		break;
        	}
        	$Comment .= chr($char);
        }
        
        return array(
        		'success' => 				true,
        		
        		'Version' => 				$Version,
        		
        		'RollGyroDirection' => 		$RollGyroDirection,
        		'PitchGyroDirection' => 	$PitchGyroDirection,
        		'YawGyroDirection' => 		$YawGyroDirection,
        		
        		'RxRollZero' => 			$RxRollZero,
        		'RxPitchZero' => 			$RxPitchZero,
        		'RxYawZero' => 				$RxYawZero,
        		
        		'Comment' => 				$Comment,
        		
        		'debug' => 					false,
        	);
    }    
}

// list of valid extensions, ex. array("jpeg", "xml", "bmp")
$allowedExtensions = array();
// max file size in bytes
$sizeLimit = 3 * 1024;

$uploader = new qqFileUploader($allowedExtensions, $sizeLimit);
$result = $uploader->handleUpload('uploads/');
// to pass data through iframe you will need to encode all html tags
echo htmlspecialchars(json_encode($result), ENT_NOQUOTES);
?>
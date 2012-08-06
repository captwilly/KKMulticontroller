<?php
require_once 'config.inc';

header("Expires: Mon, 26 Jul 1997 05:00:00 GMT"); // Date in past
header("Last-Modified: " . gmdate("D, d M Y H:i:s") . " GMT");
header("Cache-Control: no-store, no-cache, must-revalidate");
header("Cache-Control: post-check=0, pre-check=0", false);
header("Pragma: no-cache");
?>
<html>
<head>
<title>EEPROM settings file generator for KKmulticontroller</title>
<style type="text/css">
table { 
	border-collapse:collapse; 
}
td { 
	padding-right: 10px; padding-left: 10px; padding-top: 5px; padding-bottom: 5px; 
	border: 1px solid grey;
}
input {
	border: 1px solid grey;
}
select {
	border: 1px solid grey;
	background-color: white;
}
</style>
<link href="fileuploader.css" rel="stylesheet" type="text/css" />	
<script src="fileuploader.js" type="text/javascript" ></script>
<script type="text/javascript">

MaxZeroOffset = <?=MAX_ZERO_OFFSET?>;
MinZeroOffset = <?=MIN_ZERO_OFFSET?>;

function htmlspecialchars_decode (string, quote_style) {
    // Convert special HTML entities back to characters  
    // 
    // version: 1109.2015
    // discuss at: http://phpjs.org/functions/htmlspecialchars_decode
    // +   original by: Mirek Slugen
    // +   improved by: Kevin van Zonneveld (http://kevin.vanzonneveld.net)
    // +   bugfixed by: Mateusz "loonquawl" Zalega
    // +      input by: ReverseSyntax
    // +      input by: Slawomir Kaniecki
    // +      input by: Scott Cariss
    // +      input by: Francois
    // +   bugfixed by: Onno Marsman
    // +    revised by: Kevin van Zonneveld (http://kevin.vanzonneveld.net)
    // +   bugfixed by: Brett Zamir (http://brett-zamir.me)
    // +      input by: Ratheous
    // +      input by: Mailfaker (http://www.weedem.fr/)
    // +      reimplemented by: Brett Zamir (http://brett-zamir.me)
    // +    bugfixed by: Brett Zamir (http://brett-zamir.me)
    // *     example 1: htmlspecialchars_decode("<p>this -&gt; &quot;</p>", 'ENT_NOQUOTES');
    // *     returns 1: '<p>this -> &quot;</p>'
    // *     example 2: htmlspecialchars_decode("&amp;quot;");
    // *     returns 2: '&quot;'
    var optTemp = 0,
        i = 0,
        noquotes = false;
    if (typeof quote_style === 'undefined') {
        quote_style = 2;
    }
    string = string.toString().replace(/&lt;/g, '<').replace(/&gt;/g, '>');
    var OPTS = {
        'ENT_NOQUOTES': 0,
        'ENT_HTML_QUOTE_SINGLE': 1,
        'ENT_HTML_QUOTE_DOUBLE': 2,
        'ENT_COMPAT': 2,
        'ENT_QUOTES': 3,
        'ENT_IGNORE': 4
    };
    if (quote_style === 0) {
        noquotes = true;
    }
    if (typeof quote_style !== 'number') { // Allow for a single string or an array of string flags
        quote_style = [].concat(quote_style);
        for (i = 0; i < quote_style.length; i++) {
            // Resolve string input to bitwise e.g. 'PATHINFO_EXTENSION' becomes 4
            if (OPTS[quote_style[i]] === 0) {
                noquotes = true;
            } else if (OPTS[quote_style[i]]) {
                optTemp = optTemp | OPTS[quote_style[i]];
            }
        }
        quote_style = optTemp;
    }
    if (quote_style & OPTS.ENT_HTML_QUOTE_SINGLE) {
        string = string.replace(/&#0*39;/g, "'"); // PHP doesn't currently escape if more than one 0, but it should
        // string = string.replace(/&apos;|&#x0*27;/g, "'"); // This would also be useful here, but not a part of PHP
    }
    if (!noquotes) {
        string = string.replace(/&quot;/g, '"');
    }
    // Put this in last place to avoid escape being double-decoded
    string = string.replace(/&amp;/g, '&');
 
    return string;
}  

function validate() {
	valid = true;
	
	RxRollZero = parseInt(document.getElementById("RxRollZero").value);
	if(RxRollZero.toString() != document.getElementById("RxRollZero").value || 
			RxRollZero < MinZeroOffset || RxRollZero > MaxZeroOffset) {
		document.getElementById("RxRollZeroWarning").style.display = 'block';
		valid = false;
	} else {
		document.getElementById("RxRollZeroWarning").style.display = 'none';
	};

	RxPitchZero = parseInt(document.getElementById("RxPitchZero").value);
	if(RxPitchZero.toString() != document.getElementById("RxPitchZero").value || 
			RxPitchZero < MinZeroOffset || RxPitchZero > MaxZeroOffset) {
		document.getElementById("RxPitchZeroWarning").style.display = 'block';
		document.getElementById("RxPitchZeroWarning").style.visibility = 'visible';
		valid = false;
	} else {
		document.getElementById("RxPitchZeroWarning").style.display = 'none';
		document.getElementById("RxPitchZeroWarning").style.visibility = 'hidden';
	};
		
	RxYawZero = parseInt(document.getElementById("RxYawZero").value);
	if(RxYawZero.toString() != document.getElementById("RxYawZero").value || 
			RxYawZero < MinZeroOffset || RxYawZero > MaxZeroOffset) {
		document.getElementById("RxYawZeroWarning").style.display = 'block';
		valid = false;
	} else {
		document.getElementById("RxYawZeroWarning").style.display = 'none';
	};	

	if (valid) {
		document.getElementById("SaveButton").disabled = false;
	} else {
		document.getElementById("SaveButton").disabled = true;
	}

	return valid;
}

function uploadComplete (id, fileName, responseJSON) {
	if (responseJSON.success == true) {
		document.getElementById("Version").value = responseJSON.Version;
		document.getElementById("RollGyroDirection").value = responseJSON.RollGyroDirection;
		document.getElementById("PitchGyroDirection").value = responseJSON.PitchGyroDirection;
		document.getElementById("YawGyroDirection").value = responseJSON.YawGyroDirection;
		document.getElementById("RxRollZero").value = responseJSON.RxRollZero;
		document.getElementById("RxPitchZero").value = responseJSON.RxPitchZero;
		document.getElementById("RxYawZero").value = responseJSON.RxYawZero;
		document.getElementById("Comment").value = htmlspecialchars_decode(responseJSON.Comment, 'ENT_NOQUOTES');
		validate();
	}
}

function createUploader(){            
    var uploader = new qq.FileUploader({
        element: document.getElementById('file-uploader'),
        action: 'upload.php',
        onComplete: uploadComplete,
        debug: true,
    });           
}

function onLoad() {
	validate();
	createUploader();
}

</script>
</head>
<body onLoad="onLoad();">
	<form action="download.php" method="post">
	<table>
	<tr>
		<td colspan="3">
			<!-- <input type="button" value="Load EEPROM file" onclick="ajaxFileUpload();" /> -->
			<div id="file-uploader" />
		</td>
	</tr>
	<tr>
		<td colspan="2">
			Version to generate
		</td>
		<td>
			<select name="Version" id="Version">
				<option selected="selected">1</option>
			</select>
		</td>
	</tr>
	<tr>
		<td rowspan="3">Gyro reversing</td>
		<td>Roll</td>
		<td><select name="RollGyroDirection" id="RollGyroDirection"><option value="1" selected="selected">Reversed</option><option value="0">Normal</option></select></td>
	</tr>
	<tr>
		<td>Pitch</td>
		<td><select name="PitchGyroDirection" id="PitchGyroDirection"><option value="1" selected="selected">Reversed</option><option value="0">Normal</option></select></td>
	</tr>
	<tr>
		<td>Yaw</td>
		<td><select name="YawGyroDirection" id="YawGyroDirection"><option value="1">Reversed</option><option value="0" selected="selected">Normal</option></select></td>
	</tr>
	<tr>
		<td rowspan="3">Stick center</td>
		<td>Roll</td>
		<td>
			<input type="text" name="RxRollZero" id="RxRollZero" value="0" onchange="validate();" onkeyup="validate();" maxlength="4" size="4"/>
			<div id="RxRollZeroWarning" style="color: red;">It should be integer from <?=MIN_ZERO_OFFSET?> to <?=MAX_ZERO_OFFSET?></div>
		</td>
	</tr>
	<tr>
		<td>Pitch</td>
		<td>
			<input type="text" name="RxPitchZero" id="RxPitchZero" value="0" onchange="validate();" onkeyup="validate();" maxlength="4" size="4"/>
			<div id="RxPitchZeroWarning" style="color: red;">It should be integer from <?=MIN_ZERO_OFFSET?> to <?=MAX_ZERO_OFFSET?></div>
		</td>
	</tr>
	<tr>
		<td>Yaw</td>
		<td>
			<input type="text" name="RxYawZero" id="RxYawZero" value="0" onchange="validate();" onkeyup="validate();" maxlength="4" size="4"/>
			<div id="RxYawZeroWarning" style="color: red;">It should be integer from <?=MIN_ZERO_OFFSET?> to <?=MAX_ZERO_OFFSET?></div>
		</td>
	</tr>
	<tr>
		<td colspan="2">Comment</td>
		<td>
			<textarea name="Comment" id="Comment" maxlength="<?=(MAX_COMMENT_LENGTH - 1)?>">Place memo here</textarea>
		</td>
	</tr>
	<tr>
		<td colspan="3" align="right">
			<input type="submit" id="SaveButton" value="Save EEPROM file" />
		</td>
	</tr>
	</table>
	</form>
</body>
</html>
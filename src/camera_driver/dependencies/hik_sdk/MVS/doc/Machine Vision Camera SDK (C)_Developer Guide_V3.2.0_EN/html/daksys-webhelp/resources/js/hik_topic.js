
console.log("●●●●●● hik topic.js");

/*当页面滚动时，取消输入框自动补齐的Autocomplete功能*/
/* $(document).scroll(function(){
	$("#textToSearch").autocomplete("close");
});
*/

try{
	if($(top.document).width() < 510){
		// 显示 wh_side_toc
		var wh_side_toc = $('#wh_side_toc'); // topic.html 页面
		wh_side_toc.css('display', 'block');
	}
} catch (err){
	
}


var highlighted = false;
function topicOnload(){
	highlightSearchTerm();
	//水印文字
	var mark_text = $("#Trademark").val();
	if(mark_text == ""){
		return;
	}
	setTimeout( function() {
		if(navigator.appName == "Microsoft Internet Explorer"&& navigator.appVersion.match(/11./i)!="11."){
			waterMark$(mark_text);
		} else {
			waterMarkNotIe$(mark_text);
		}
	}, 500);
}

function highlightSearchTerm() {
    if (highlighted) {
        return;
    }
    try {
        var $body = $('.wh_topic_content');
        var $relatedLinks = $('.wh_related_links');
		var $childLinks = $('.wh_child_links');

        // Test if highlighter library is available
        if (typeof $body.removeHighlight != 'undefined') {
			console.log("highlightSearchTerm()");
            $body.removeHighlight();
            $relatedLinks.removeHighlight();
			//高亮关键字
            var hlParameter = getParameter('hl');
            if (hlParameter != undefined) {
                var jsonString = decodeURIComponent(String(hlParameter));
                console.log("jsonString: ", jsonString);
                if (jsonString !== undefined && jsonString != "") {
                    var words = jsonString.split(',');
                    for (var i = 0; i < words.length; i++) {
                        console.log('highlight(' + words[i] + ');');
                        $body.highlight(words[i]);
                        $relatedLinks.highlight(words[i]);
                        $childLinks.highlight(words[i]);
                    }
                }
            }
        } else {
            // JQuery highlights library is not loaded
        }
    }
    catch (e) {
        console.log (e);
    }
    highlighted = true;
}
function getParameter(parameter) {
    var whLocation = "";

    try {
        whLocation = window.location;
        var p = parseUri(whLocation);
    } catch (e) {
        debug(e);
    }

    return p.queryKey[parameter];
}


function waterMark$(watermark){
	console.log("waterMark()");
	$("p[name='p1$']").remove();
	var winwidth$ = document.body.scrollWidth-17;
	var winheight$ = document.body.scrollHeight;
	$("body").append("<p id='waterSum_11' name='p1$' class='cover_through cover js-click-to-alert'>"+watermark+"</p>");
	var fleft = Number($('#waterSum_11').css("margin-left").substring(0,$('#waterSum_11').css("margin-left").indexOf('p')));
	var ftop = Number($('#waterSum_11').css("margin-top").substring(0,$('#waterSum_11').css("margin-top").indexOf('p')));
	var perWidth = $("#waterSum_11").width();
	var perHeight = Number('180px'.substring(0,'180px'.indexOf('p')));
	var lines = parseInt(winwidth$/(perWidth + fleft*2));
	var rows = Math.round(winheight$/perHeight);
	console.log("ROW:" + rows + " winheight:"+winheight$ + " perHeight:" + perHeight);
	var totalPWidth = perWidth*lines;
	var totalSpace = winwidth$-totalPWidth;
	var perSpace = parseInt(totalSpace/(lines+1));
	$('#waterSum_11').css("margin-left",perSpace);
	for(var i=1;i<=rows;i++) {
		for(var j=1;j<=lines;j++){
			if(i==1){
				if(j<=lines-1){
					var p = "<p id='waterSum_"+i+""+(j+1)+"' name='p1$' class='cover_through cover js-click-to-alert'>"+watermark+"</p>";
					var ileft = $('#waterSum_'+i+''+j).css("margin-left").substring(0,$('#waterSum_'+i+''+j).css("margin-left").indexOf('p'));
					var itop = $('#waterSum_11').css("margin-top").substring(0,$('#waterSum_11').css("margin-top").indexOf('p'));
					$("body").append(p);
					$('#waterSum_'+i+''+(j+1)).css("margin-left",Number(ileft)+Number(perWidth)+perSpace);
					$('#waterSum_'+i+''+(j+1)).css('margin-top',itop);
				}
			}else{
				var p = "<p id='waterSum_"+i+""+j+"' name='p1$' class='cover_through cover js-click-to-alert'>"+watermark+"</p>";
				var ileft = $('#waterSum_'+(i-1)+''+j).css("margin-left").substring(0,$('#waterSum_'+(i-1)+''+j).css("margin-left").indexOf('p'));
				var itop =  $('#waterSum_'+(i-1)+''+j).css("margin-top").substring(0,$('#waterSum_'+(i-1)+''+j).css("margin-top").indexOf('p'));
				$("body").append(p);
				$('#waterSum_'+i+''+j).css("margin-left",Number(ileft));
				$('#waterSum_'+i+''+j).css('margin-top',Number(itop)+Number(perHeight));
			}
		}
	}
	passThrough();
	
}
function waterMarkNotIe$(watermark){
	console.log("waterMark Not Ie()");
	var winwidth$ = document.body.clientWidth;
	var winheight$ = document.body.scrollHeight;
	var waterSum$ = 200;
	var oldleft$=0;
	var maxI$=0;
	var k$=0;
	$("body").append("<div class='cover-Blink-area'> </div>");
	$('.cover-Blink-area').css('height', winheight$+'px');
	for( var i=1;i<=waterSum$;i++) {
		$(".cover-Blink-area").append("<p id='waterSum_" +i+"' class='cover_through cover-Blink js-click-to-alert'>"+watermark+"</p>");
		var left = Number(document.getElementById("waterSum_" +i).offsetLeft);
		if(left>oldleft$) {
			oldleft$ = left;
			maxI$ = i;
		}
		if (left<oldleft$&&k$==0){
			var top = $("#waterSum_1").css("margin-top").substring(0,$("#waterSum_1").css("margin-top").indexOf('p'));
			var bottom = $("#waterSum_1").css("margin-bottom").substring(0,$("#waterSum_1").css("margin-bottom").indexOf('p'));
			var pHeight = $("#waterSum_1").height();
			var totalHeight = Number(top)+Number(pHeight)+Number(bottom);
			var Hnum = Math.round(winheight$/(totalHeight/1.3));
			waterSum$ = Hnum*maxI$;
			k$++;
		}
	}
}
window.onresize = function(){
	//水印文字
	var mark_text = $("#Trademark").val();
	if(mark_text == ""){
		return;
	}
	//删除水印
	$(".cover-Blink-area").remove();
	//添加水印
	if(navigator.appName == "Microsoft Internet Explorer"&& navigator.appVersion.match(/11./i)!="11."){
		waterMark$(mark_text);
	} else {
		waterMarkNotIe$(mark_text);
	}
}
function passThrough() {
	$(".cover").mouseenter(function(){
		$(this).stop(true).fadeOut().delay(1500).fadeIn(50);
	});
}

function debug(msg, object){
	console.log(msg, object); 
}
function executeQuery(){
	$('#searchForm').submit(); 
	return true; 
}
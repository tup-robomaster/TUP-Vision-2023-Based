

/*当页面滚动时，取消输入框自动补齐的Autocomplete功能*/
$(document).scroll(function(){
	$("#textToSearch").autocomplete("close");
});

/*记录li_has_children的li_open和li_close*/
var liStatus;
if($.cookie('li_status') != null){
	liStatus = str2array($.cookie('li_status'));
	//console.log(liStatus);
	if(liStatus.length == $('.li_has_children').length){
		for(var i = 0; i < $('.li_has_children').length; i++){
			if(liStatus[i] == 'li_open'){
				setLiStatusOpen($('.li_has_children').eq(i));
			}else if(liStatus[i] == 'li_close'){
				setLiStatusClose($('.li_has_children').eq(i));
			}
		}
	}else{
		liStatus = new Array();
		$('.li_has_children').each(function(){
			liStatus.push('li_close');
			setLiStatusClose($(this));
		});
		$.cookie('li_status', array2str(liStatus), { expires: 7 });
	}
}else{
	liStatus = new Array();
	$('.li_has_children').each(function(){
		liStatus.push('li_close');
		setLiStatusClose($(this));
	});
	$.cookie('li_status', array2str(liStatus), { expires: 7 });
}

/*增加li_before的点击事件*/
$('.li_before').unbind('click').bind('click', function(){
	var liPosition = $('.li_has_children').index($(this).parent().parent());
	if($(this).parent().parent().hasClass('li_close')){
		$(this).parent().parent().removeClass('li_close');
		$(this).parent().parent().addClass('li_open');
		liStatus[liPosition] = 'li_open';
	}else if($(this).parent().parent().hasClass('li_open')){
		$(this).parent().parent().addClass('li_close');
		$(this).parent().parent().removeClass('li_open');
		liStatus[liPosition] = 'li_close';
	}
	$.cookie('li_status', array2str(liStatus), { expires: 7 });
});

/*监听窗口的滚动事件*/
var startX, startY;  
document.addEventListener('touchstart', function(ev){
    startX = ev.touches[0].pageX;
    startY = ev.touches[0].pageY;
}, false);
document.addEventListener('touchend',function (ev) {  
    var endX, endY;  
    endX = ev.changedTouches[0].pageX;  
    endY = ev.changedTouches[0].pageY;  
    var direction = GetSlideDirection(startX, startY, endX, endY);
    switch(direction){
        case 0:
            break;
        case 1:
            // 手指向上滑动
            //$('.wh_header').css('display', 'none');
            //$('.wh_main_page_search').css('display', 'none');
            //$('.wh_search_input').css('display', 'none');
            var scrollTop = 0;
            var scrollTimer = setInterval(function(){
				if(scrollTop != $(document).scrollTop()){
					scrollTop = $(document).scrollTop();
				}else{
					scrollTop = $(document).scrollTop();
					clearInterval(scrollTimer);
					//alert(scrollTop);
					if($(document).scrollTop() > 60){
		            	$('.wh_header').css('display', 'none');
		            	$('.wh_main_page_search').css('display', 'none');
		            	$('.wh_search_input').css('display', 'none');
		            }
				}
			}, 50);
            break;
        case 2:
            // 手指向下滑动
            //alert(endScrollTop());
            var scrollTop = 0;
            var scrollTimer = setInterval(function(){
				if(scrollTop != $(document).scrollTop()){
					scrollTop = $(document).scrollTop();
				}else{
					scrollTop = $(document).scrollTop();
					clearInterval(scrollTimer);
					//alert(scrollTop);
					if($(document).scrollTop() <= 60){
		            	$('.wh_header').css('display', 'block');
		            	$('.wh_main_page_search').css('display', 'block');
		            	$('.wh_search_input').css('display', 'block');
		            }
				}
			}, 50);
            break;
    }
}, false);
function setLiStatusOpen(obj){
	obj.removeClass('li_close');
	obj.addClass('li_open');
}
function setLiStatusClose(obj){
	obj.removeClass('li_open');
	obj.addClass('li_close');
}
function array2str(array){
	var str = "";
	for(var i = 0; i < array.length; i++){
		if(i != array.length - 1){
			str = str + array[i] + "-";
		}else{
			str = str + array[i];
		}
	}
	return str;
}
function str2array(str){
	var array = new Array();
	for(var i = 0; i < str.split('-').length; i++){
		array.push(str.split('-')[i]);
	}
	return array;
}
function GetSlideDirection(startX, startY, endX, endY) {  
    var dy = startY - endY;  
    var dx = endX - startX;  
    var result = 0; 
    if(dy > 0){//向上滑动
        return 1;
    }else{//向下滑动
        return 2;
    }
    if(dx > 0){//向右滑动
    	return 3;
    }else{//向左滑动
    	return 4;
    } 
}

// desktop 显示目录的全部
setDesktopMenuMove();
function setDesktopMenuMove(){
    $('.desktop_left').find('a').unbind('mouseover').bind('mouseover', function(){
        var flag = isEllipsis($(this)[0]);
        if(flag){
            $('.menu-tempdiv').html($(this).html());
            $('.menu-tempdiv').removeClass('hidden');
            $('.menu-tempdiv').addClass('show');
            $('.menu-tempdiv').css('top', $(this).offset().top + 29 - $('body,html').scrollTop() + 'px');
            $('.menu-tempdiv').css('left', $(this).offset().left + 'px');
            /*if($(this).parent().hasClass('li-1')){
                $('.menu-tempdiv').css('left', $(this).offset().left + 36 + 'px');
            }else if($(this).parent().hasClass('li-2')){
                $('.menu-tempdiv').css('left', $(this).offset().left + 72 + 'px');
            }else if($(this).parent().hasClass('li-3')){
                $('.menu-tempdiv').css('left', $(this).offset().left + 108 + 'px');
            }*/
        }
    })
    $('.desktop_left').find('a').unbind('mouseout').bind('mouseout', function(){
        $('.menu-tempdiv').removeClass('show');
        $('.menu-tempdiv').addClass('hidden');
    })
}
function isEllipsis(dom) {
    var checkDom = dom.cloneNode(), parent, flag;
    checkDom.style.width = dom.offsetWidth + 'px';
    checkDom.style.height = dom.offsetHeight + 'px';
    checkDom.style.overflow = 'auto';
    checkDom.style.position = 'absolute';
    checkDom.style.zIndex = -1;
    checkDom.style.opacity = 0;
    checkDom.style.whiteSpace = "nowrap";
    checkDom.innerHTML = dom.innerHTML;
 
    parent = dom.parentNode;
    parent.appendChild(checkDom);
    flag = checkDom.scrollWidth > checkDom.offsetWidth;
    parent.removeChild(checkDom);
    return flag;
}


// Mobile
function bindClick_Mobile(){
	$('.wh_toggle_button').bind('click', function(event){
		if($('.wh_top_menu_and_indexterms_link').hasClass('in')){
			$('.wh_toggle_button').css('background-color', '#2d2d2d');
			$('.wh_toggle_button').find('.icon-bar').css('background-color', 'white');
		}else{
			$('.wh_toggle_button').css('background-color', '#ddd');
			$('.wh_toggle_button').find('.icon-bar').css('background-color', '#333');
		}
	});
	$('#content_area').bind('click', function(){
		if($('.wh_top_menu_and_indexterms_link').hasClass('in')){
			$('.wh_toggle_button').click();
		}
	});
	$('.wh_header').bind('click', function(){
		if($('.wh_top_menu_and_indexterms_link').hasClass('in')){
			$('.wh_toggle_button').click();
		}
	});
	$('.wh_search_input').bind('click', function(){
		if($('.wh_top_menu_and_indexterms_link').hasClass('in')){
			$('.wh_toggle_button').click();
		}
	});
    
}
if($(document).width() < 767){
	console.log("●●●●●●手机端 index.html页面打开菜单");
	/* 隐藏 container_desktop*/
	var containerDesktop = $('.container_desktop');
	containerDesktop.css('display', 'none');
	
	/*wh_top_menu_and_indexterms_link的最大高度为页面的最大高度*/
	$('.wh_top_menu_and_indexterms_link').eq(0).css('max-height', $(window).height() * 0.5 + 'px');
	/*修改默认的手机端右上角目录按钮事件*/
	bindClick_Mobile();
	
		
}else{
	console.log("●●●●●●PC端 index.html页面跳转到第一个topic");
	/* 隐藏 wh_content_flex_container*/
	var containerMobile = $('.container_mobile');
	var containerDesktop = $('.container_desktop');
	containerMobile.css('display', 'none');
	containerDesktop.css('display', 'none');
	// 隐藏 side_toc
	var wh_side_toc = $('#wh_side_toc');
	wh_side_toc.css('display', 'none');
	// index.html页面跳转到第一个topic
	var home_url = window.location.href;
	if(home_url.indexOf('index.html') > 0 || home_url.charAt(home_url.length-1)=='/'){
		var a = $('.desktop_left').eq(0).children('.menu_ul').eq(0).children('li').eq(0).find('a').eq(0);
		if(a.attr('href') != null && a.attr('href') != ''){
			window.location.href = a.attr('href');
		}
	}
}

// Add some Bootstrap classes when document is ready
var highlighted = false;

$(document).ready(function () {
	console.log("●●●●●● wt_default >> document.ready");
	
	var pageWidth = document.body.clientWidth;
	//search.html
    try {
        var searchQuery = getParameter('searchQuery');
        if (searchQuery!='' && searchQuery!==undefined && searchQuery!='undefined') {
			searchQuery = decodeURIComponent(searchQuery);
			searchQuery = searchQuery.replace(/\+/g, " ");
            $('#textToSearch').val(searchQuery);
            executeQuery();
        }
    } catch (e) {
        debug(e);
    }
	
    // If we have a contextID, we must to redirect to the corresponding topic
    var contextId = getParameter('contextId');
    var appname = getParameter('appname');

    if ( contextId != undefined && contextId != "") {
        var scriptTag = document.createElement("script");
        scriptTag.type = "text/javascript";
        scriptTag.src = "context-help-map.js";
        document.getElementsByTagName('head')[0].appendChild(scriptTag);

        var ready = setInterval(function () {
			if (helpContexts != undefined) {
				for(var i = 0; i < helpContexts.length; i++) {
					var ctxt = helpContexts[i];
					if (contextId == ctxt["appid"] && (appname == undefined || appname == ctxt["appname"])) {
						var path = ctxt["path"];
						if (path != undefined) {
							window.location = path;
						}
						break;
					}
				}
				clearInterval(ready);
			}
        }, 100);
    }

    // Navigational links and print
    $('#topic_navigation_links .navprev>a').addClass("glyphicon glyphicon-arrow-left");
    $('#topic_navigation_links .navnext>a').addClass("glyphicon glyphicon-arrow-right");
    $('.wh_print_link a').addClass('glyphicon glyphicon-print');
	
    var containerBody = $('#container_body');
	var containerBodyLeft =  $('#container_body_left');
	var containerBodyRight =  $('#container_body_right');
	var containerBodyArea =  $('#contentArea');
	var container_mobile = $('.container_mobile');
	
    if (pageWidth < 510) {
		console.log("●●●●●●动态改变container_body内部CSS，适应手机");
		//手机端CSS	
		$(".wh_header_flex_container").css('padding','0 1em');
		containerBodyLeft.css('display', 'none');
		containerBodyLeft.removeClass(' desktop_left col-sm-3');
	
		containerBodyRight.removeClass(' desktop_right col-sm-9');
	
		containerBodyArea.removeClass(' col-lg-12 col-md-12 col-sm-12 col-xs-12');
		containerBodyArea.addClass(' col-lg-8 col-md-8 col-sm-8 col-xs-12');
		
		containerBody.removeClass(' container_desktop wh_content_flex_container');
		containerBody.addClass(' container_mobile');
		container_mobile.css('flex-direction', 'column');
    } else {
		console.log("●●●●●●动态改变container_body内部CSS，适应PC");
		//desktop端CSS	
		$(".wh_header_flex_container").css('padding','0 calc(5% + 30px)');
		containerBodyLeft.css('display', 'block');
		containerBodyLeft.css('padding', '0px');
		containerBodyLeft.addClass(' desktop_left col-sm-3');
	
		containerBodyRight.addClass(' desktop_right');
	
		containerBodyArea.removeClass(' col-lg-8 col-md-8 col-sm-8 col-xs-12');
		containerBodyArea.addClass(' col-lg-12 col-md-12 col-sm-12 col-xs-12');
		
		containerBody.css('display', 'block');
		containerBody.removeClass(' container_mobile');
		containerBody.addClass(' container_desktop wh_content_flex_container');
		container_mobile.css('flex-direction', 'initial');
		
	}
	
	
	/*根据url添加目录的active类*/
	/* var url = location.href;
	var href = url.split('/')[url.split('/').length - 1];
	console.log("●●●●●●根据url添加目录的active", $('.desktop_left').length);
	if($('.desktop_left').length != 0){
		$('.desktop_left').find('li').each(function(){
			if(href == $(this).find('a').attr('href')){
				$(this).addClass('active');
			}
		});
	} */

    $(".wh_main_page_toc .wh_main_page_toc_accordion_header").click(function(event) {
        if ($(this).hasClass('expanded')) {
            $(this).removeClass("expanded");
        } else {
            $(".wh_main_page_toc .wh_main_page_toc_accordion_header").removeClass("expanded");
            $(this).addClass("expanded");
        }

        event.stopImmediatePropagation();
        return false;
    });

    $(".wh_main_page_toc a").click(function(event) {
        event.stopImmediatePropagation();
    });


	
    var dirAttr = $('html').attr('dir');
    var rtlEnabled = false;
    if (dirAttr=='rtl') {
        rtlEnabled = true;
    }

    var isTouchEnabled = false;
    try {
        if (document.createEvent("TouchEvent")) {
           isTouchEnabled = true;
        }
    } catch (e) {
        //debug(e);
    }
    //alert(isTouchEnabled);
    if ($(window).width() >= 510 && !isTouchEnabled) {
	    $('.wh_top_menu').find('li').hover(function(){
	        var firstLevelElementWidth = $('.wh_top_menu>ul>li:hover').width();
	        var totalWidth = 0;
	        $.each($('.wh_top_menu>ul li:hover'), function() {
	            totalWidth+=parseInt($(this).width());
	        });
	        var offsetLeft = parseInt($(this).offset().left);
	        var childWidth = 0;
	        try {
	            childWidth = parseInt($(this).children('ul').width());
	        } catch (e) {
	            debug(e);
	        }
	        totalWidth += childWidth - firstLevelElementWidth;
	        var index = $('.wh_top_menu ul').index($(this).parent('ul'));
	        if (!rtlEnabled) {
	            var neededWidth = offsetLeft + totalWidth;
	            if (neededWidth > parseInt($(window).width()) && index != 0) {
	                $(this).children('ul').css('right', '100%');
	                $(this).children('ul').css('left', 'auto');
	            } else if (index != 0) {
	                $(this).children('ul').css('right', 'auto');
	                $(this).children('ul').css('left', '100%');
	            }
	        } else {
	            var leftPositionNeeded = offsetLeft - totalWidth + childWidth;
	            if (leftPositionNeeded < 0 && index != 0) {
	                $(this).children('ul').css('right', 'auto');
	                $(this).children('ul').css('left', '100%');
	            } else if (index != 0) {
	                $(this).children('ul').css('right', '100%');
	                $(this).children('ul').css('left', 'auto');
	            }
	        }
	    });
    }

    highlightSearchTerm();
	
});

/**
 * @description Log messages and objects value into browser console
 */
function debug(message, object) {
    object = object || "";
    console.log(message, object);
}

/**
 * @description Highlight searched words
 */
function highlightSearchTerm() {
    if (highlighted) {
        return;
    }
    //debug("highlightSearchTerm()");
    try {
        var $body = $('.wh_topic_content');
        var $relatedLinks = $('.wh_related_links');
		var $childLinks = $('.wh_child_links');

        // Test if highlighter library is available
        if (typeof $body.removeHighlight != 'undefined') {
            $body.removeHighlight();
            $relatedLinks.removeHighlight();

            var hlParameter = getParameter('hl');
            if (hlParameter != undefined) {
                var jsonString = decodeURIComponent(String(hlParameter));
                debug("jsonString: ", jsonString);
                if (jsonString !== undefined && jsonString != "") {
                    var words = jsonString.split(',');
                    for (var i = 0; i < words.length; i++) {
                        debug('highlight(' + words[i] + ');');
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
        debug (e);
    }
    highlighted = true;
}

/**
 * @description Returns all available parameters or empty object if no parameters in URL
 * @return {Object} Object containing {key: value} pairs where key is the parameter name and value is the value of parameter
 */
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

/**
 * Open the link from top_menu when the current group is expanded.
 */
$(".wh_top_menu li").click(function (event) {
	console.log("●● wh_top_menu li click ");

    $(".wh_top_menu li").removeClass('active');
    $(this).addClass('active');
    $(this).parents('li').addClass('active');
    event.stopImmediatePropagation();
});

$(".wh_top_menu a").click(function (event) {
	console.log("●● wh_top_menu a click ");

    var isTouchEnabled = false;
    try {
        if (document.createEvent("TouchEvent")) {
            isTouchEnabled = true;
        }
    } catch (e) {
        debug(e);
    }
    if ($(window).width() < 510 || isTouchEnabled) {
        var areaExpanded = $(this).closest('li');
        if (areaExpanded.hasClass('active') || areaExpanded.find('li').length == 0) {
            window.location = $(this).attr("href");
            event.preventDefault();
            event.stopImmediatePropagation();
            return false;
        } else {
            event.preventDefault();
        }
    } else {
        return true;
    }
});

function openTopicInMobileFrame(alink){
	console.log("●●●●●● 手机端 frame中打开页面");
	var href = alink.attr('data-href');
	$("#frame_mobile").attr("src", href);
	console.log("●●●●●●手机端 隐藏首页菜单");
	$("#mobile_menu").css("display", "none");
}
function openTopicInDesktopFrame(alink){
	console.log("menu active");
	$(".menu_ul li").removeClass('active');
	alink.parents('li').addClass('active');
	
	console.log("●●●●●● Desktop frame中打开页面");
	var href = alink.attr('data-href');
	$("#frame_desktop").attr("src", href);
}

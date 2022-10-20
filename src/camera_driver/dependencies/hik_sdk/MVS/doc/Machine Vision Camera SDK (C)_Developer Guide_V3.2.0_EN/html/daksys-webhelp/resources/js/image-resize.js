$(function () {
    var imgDom = $('img.image');
    var i = 0, l = imgDom.length, width, height, className, scale, isSpec;
    for (; i < l; i++) {
        className = imgDom[i].className;
        width = imgDom[i].getAttribute('width');
        height = imgDom[i].getAttribute('height');
        scale = imgDom[i].getAttribute('scale');
        
        if(className) {
            switch (true) {
                case (className.indexOf('spec_icon') > -1 &&  height == '60'):
                    imgDom[i].setAttribute('height', '600px');
                    imgDom[i].setAttribute('width', ''); /* width is invalid  */
					isSpec = true;
                    continue;
                case className.indexOf('spec_icon') > -1:
                    imgDom[i].setAttribute('height', '300px');
                    imgDom[i].setAttribute('width', ''); /* width is invalid  */
                    isSpec = true;
                    continue;
                case className.indexOf('bigicon') > -1 && isSpec == true:
                    imgDom[i].setAttribute('height', '80px');
                    imgDom[i].setAttribute('width', '');
                    continue;
                case className.indexOf('icon') > -1 && isSpec == true:
                    imgDom[i].setAttribute('height', '80px');
                    imgDom[i].setAttribute('width', '');
                    continue;
                case className.indexOf('bigicon') > -1:
                    imgDom[i].setAttribute('height', '40px');
                    imgDom[i].setAttribute('width', '');
                    continue;
                case className.indexOf('icon') > -1:
                    imgDom[i].setAttribute('height', '20px');
                    imgDom[i].setAttribute('width', '');
                    continue;
            }
        }
        if(width) {
            switch (true) {
                case width >= 1417:
                    imgDom[i].setAttribute('width', '700px');
                    imgDom[i].setAttribute('height', '');
                    break;
                case width >= 945 && width < 1417:
                    imgDom[i].setAttribute('width', '300px');
                    imgDom[i].setAttribute('height', '');
                    break;
                case width >= 591 && width < 945:
                    imgDom[i].setAttribute('width', '188px');
                    imgDom[i].setAttribute('height', '');
                    break;
                case width >= 354 && width < 591:
                    imgDom[i].setAttribute('width', '122px');
                    imgDom[i].setAttribute('height', '');
                    break;
                case width >= 0 && width < 354:
                    imgDom[i].setAttribute('width', '75px');
                    imgDom[i].setAttribute('height', '');
                    break;
            }
        } else if(height) {
            switch (true) {
                case height >= 2362:
                    if (BrowserDetect.browser=='Firefox' ){
                        imgDom[i].setAttribute('style', 'max-height:750px');
                    } else {
						imgDom[i].style.cssText = "max-height:750px";
					}
                    imgDom[i].setAttribute('height', '');
                    imgDom[i].setAttribute('width', '');
                    break;
                case height >= 1417:
                    if (BrowserDetect.browser=='Firefox' ){
                        imgDom[i].setAttribute('style', 'max-height:450px');
                    } else {
						imgDom[i].style.cssText = "max-height:450px";
					}
                    imgDom[i].setAttribute('height', '');
                    imgDom[i].setAttribute('width', '');
                    break;
                case height >= 945:
					if (BrowserDetect.browser=='Firefox' ){
                        imgDom[i].setAttribute('style', 'max-height:300px');
                    } else {
						imgDom[i].style.cssText = "max-height:300px";
					}
                    imgDom[i].setAttribute('height', '');
                    imgDom[i].setAttribute('width', '');
                    break;
                case height >= 500:
                    if (BrowserDetect.browser=='Firefox' ){
                        imgDom[i].setAttribute('style', 'max-height:150px');
                    } else {
						imgDom[i].style.cssText = "max-height:150px";
					}
                    imgDom[i].setAttribute('height', '');
                    imgDom[i].setAttribute('width', '');
                    break;
                case height >= 119:
                    if (BrowserDetect.browser=='Firefox' ){
                        imgDom[i].setAttribute('style', 'max-height:75px');
                    } else {
						imgDom[i].style.cssText = "max-height:75px";
					}
                    imgDom[i].setAttribute('height', '');
                    imgDom[i].setAttribute('width', '');
                    break;
                case height >= 0:
                    if (BrowserDetect.browser=='Firefox' ){
                        imgDom[i].setAttribute('style', 'max-height:30px');
                    } else {
						imgDom[i].style.cssText = "max-height:30px";
					}
                    imgDom[i].setAttribute('height', '');
                    imgDom[i].setAttribute('width', '');
                    break;
            }
        } else if(scale) {
            imgDom[i].setAttribute('scale', '');
            imgDom[i].setAttribute('height', '');
            imgDom[i].setAttribute('width', scale + '%');
        }
    }
    imgDom = null;
})


/* 后退操作 */
var curWwwPath = window.document.location.href;
var pathName = window.document.location.pathname;
var pos = curWwwPath.indexOf(pathName);
var localhostPaht = curWwwPath.substring(0,pos);
var host_address = localhostPaht + pathName;
var aaa = window.location;
	
var initSrc;
window.onload = firstTimeLoad
function firstTimeLoad() {
    initSrc = document.getElementById("frame_desktop").src;
	if(localhostPaht != "file://"){
		initSrc = initSrc.substring(host_address.length);
	} else {
		initSrc = initSrc.substring(initSrc.lastIndexOf("/") + 1);
	}
	pushHistory(initSrc);//src 放入 history
}

window.onpopstate = popState;
function popState(event){
	var guid_html = "";
    if(event.state){
		history.go(-1);//后退一步
		guid_html = event.state.url;
		console.log("popState(event): "+guid_html);
    } else {
		//退到初始页面
		document.getElementById("frame_desktop").src = initSrc;
		guid_html = initSrc;
		console.log("popState 退到初始页面 "+guid_html);
	}
	
	$(".menu_ul li").removeClass('active');
	//高亮左侧菜单 desktop_left menu_ul
	$('.desktop_left li').each(function(){
		var a = $(this).find("a").eq(0);
		if(a.attr('data-href') != null && a.attr('data-href') == guid_html){
			//激活
			active_li_ancestor($(this));
			//判断是否需要滚动
			var scrollTopMenu = $(this).parents('.desktop_left_menu_ul').eq(0).scrollTop();
			var liOffsetTop = $(this).parent().offset().top;
			var menuHeight = $(this).parents('.desktop_left_menu_ul').eq(0).height();
			if (menuHeight < liOffsetTop) {
				$(this).parents('.desktop_left_menu_ul').eq(0).scrollTop(scrollTopMenu + liOffsetTop - 60)
			}
		}
	});
		
}
/* 激活li, 打开上层菜单 */
function active_li_ancestor(li_obj){
	li_obj.addClass('active');
	var parent_li = li_obj.parent().parent();
	if(parent_li[0].nodeName == "LI"){
		parent_li.removeClass('li_close');
		parent_li.addClass('li_open');
		active_li_ancestor(parent_li);
	}
}

function pushHistory(src) {
	console.log("pushHistory():" + src);
	if(src != ""){
		var state = {
			title: "title", //浏览器忽略此参数
			url: src  //浏览器显示此地址
		};
		window.history.pushState(state, "title", "");//添加浏览历史
	}
	
}
//添加后退事件监视器
/* window.addEventListener("popstate", function(event) {
	console.log("屏蔽后退事件监视器");
	if(event.state){
		console.log("event.state");
	} else {
		console.log("event.state = null");
	}
}, false); */
	
function iframeOnload(){
	var src = document.getElementById("frame_desktop").src;
	if(localhostPaht != "file://"){
		src = src.substring(host_address.length);
	} else {
		src = src.substring(src.lastIndexOf("/") + 1);
	}
	pushHistory(src);//src 放入 history
}
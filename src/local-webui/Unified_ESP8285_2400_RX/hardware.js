document.addEventListener('DOMContentLoaded',onReady,false);function _(el){return document.getElementById(el);}
function onReady(){[['icon-input','Digital Input'],['icon-output','Digital Output'],['icon-analog','Analog Input'],['icon-pwm','PWM Output']].forEach((t)=>{let imgs=document.getElementsByClassName(t[0]);[...imgs].forEach(i=>i.title=t[1]);});if(window.File&&window.FileList&&window.FileReader){initFiledrag();}
loadData();}
function loadData(){xmlhttp=new XMLHttpRequest();xmlhttp.onreadystatechange=function(){if(this.readyState===4&&this.status===200){const data=JSON.parse(this.responseText);updateHardwareSettings(data);}};xmlhttp.open('GET','/hardware.json',true);xmlhttp.setRequestHeader('Content-type','application/x-www-form-urlencoded');xmlhttp.send();}
function submitHardwareSettings(){const xhr=new XMLHttpRequest();xhr.open('POST','/hardware.json');xhr.setRequestHeader('Content-Type','application/json');const formData=new FormData(_('upload_hardware'));xhr.send(JSON.stringify(Object.fromEntries(formData),function(k,v){if(v==='')return undefined;if(_(k)&&_(k).type==='checkbox'){return v==='on';}
if(_(k)&&_(k).classList.contains('array')){const arr=v.split(',').map((element)=>{return Number(element);});return arr.length===0?undefined:arr;}
return isNaN(v)?v:+v;}));xhr.onreadystatechange=function(){if(this.readyState===4&&this.status===200){cuteAlert({type:'question',title:'Upload Succeeded',message:'Reboot to take effect',confirmText:'Reboot',cancelText:'Close'}).then((e)=>{if(e==='confirm'){const xhr=new XMLHttpRequest();xhr.open('POST','/reboot');xhr.setRequestHeader('Content-Type','application/json');xhr.onreadystatechange=function(){};xhr.send();}});}};return false;}
function updateHardwareSettings(data){for(const[key,value]of Object.entries(data)){if(_(key)){if(_(key).type==='checkbox'){_(key).checked=value;}else{if(Array.isArray(value))_(key).value=value.toString();else _(key).value=value;}}}
if(data.customised)_('custom_config').style.display='block';}
function fileDragHover(e){e.stopPropagation();e.preventDefault();if(e.target===_('filedrag'))e.target.className=(e.type==='dragover'?'hover':'');}
function fileSelectHandler(e){fileDragHover(e);const files=e.target.files||e.dataTransfer.files;_('upload_hardware').reset();for(const f of files){parseFile(f);}}
function parseFile(file){const reader=new FileReader();reader.onload=function(e){const data=JSON.parse(e.target.result);updateHardwareSettings(data);};reader.readAsText(file);}
function initFiledrag(){const fileselect=_('fileselect');const filedrag=_('filedrag');fileselect.addEventListener('change',fileSelectHandler,false);const xhr=new XMLHttpRequest();if(xhr.upload){filedrag.addEventListener('dragover',fileDragHover,false);filedrag.addEventListener('dragleave',fileDragHover,false);filedrag.addEventListener('drop',fileSelectHandler,false);filedrag.style.display='block';}}
const elrsApi=(()=>{const defaultHost='http://10.0.0.1';const storageKey='elrs-api-base';function normalize(host){if(!host)return defaultHost;host=host.trim();if(!/^https?:\/\//i.test(host))host=`http://${host}`;return host.replace(/\/+$/,'');}
function base(){const params=new URLSearchParams(window.location.search);const requested=params.get('host')||params.get('api');if(requested){const value=normalize(requested);localStorage.setItem(storageKey,value);return value;}
return normalize(localStorage.getItem(storageKey)||defaultHost);}
function url(path){if(!path||/^(https?:|blob:|data:)/i.test(path))return path;if(path[0]==='/')return base()+path;return`${base()}/${path}`;}
function rewriteLinks(){document.querySelectorAll('a[href]').forEach((link)=>{const href=link.getAttribute('href');if(href&&(href[0]==='/'||href==='firmware.bin')){link.setAttribute('href',url(href));}});document.querySelectorAll('form[action]').forEach((form)=>{const action=form.getAttribute('action');if(action&&action[0]==='/'){form.setAttribute('action',url(action));}});}
return{base,url,rewriteLinks};})();(()=>{const open=XMLHttpRequest.prototype.open;XMLHttpRequest.prototype.open=function(method,url,async,user,password){return open.call(this,method,elrsApi.url(url),async,user,password);};document.addEventListener('DOMContentLoaded',elrsApi.rewriteLinks,false);})();function postWithFeedback(title,msg,url,getdata,success){return function(e){e.stopPropagation();e.preventDefault();xmlhttp=new XMLHttpRequest();xmlhttp.onreadystatechange=function(){if(this.readyState===4){if(this.status===200){if(success)success();cuteAlert({type:'info',title:title,message:this.responseText});}else{cuteAlert({type:'error',title:title,message:msg});}}};xmlhttp.open('POST',url,true);if(getdata)data=getdata(xmlhttp);else data=null;xmlhttp.send(data);};}
function cuteAlert({type,title,message,buttonText='OK',confirmText='OK',cancelText='Cancel',closeStyle,}){return new Promise((resolve)=>{setInterval(()=>{},5000);const body=document.querySelector('body');let closeStyleTemplate='alert-close';if(closeStyle==='circle'){closeStyleTemplate='alert-close-circle';}
let btnTemplate=`<button class="alert-button ${type}-bg ${type}-btn mui-btn mui-btn--primary">${buttonText}</button>`;if(type==='question'){btnTemplate=`
<div class="question-buttons">
  <button class="confirm-button error-bg error-btn mui-btn mui-btn--danger">${confirmText}</button>
  <button class="cancel-button question-bg question-btn mui-btn">${cancelText}</button>
</div>
`;}
let svgTemplate=`
<svg class="alert-img" xmlns="http://www.w3.org/2000/svg" fill="#fff" viewBox="0 0 52 52" xmlns:v="https://vecta.io/nano">
<path d="M26 0C11.664 0 0 11.663 0 26s11.664 26 26 26 26-11.663 26-26S40.336 0 26 0zm0 50C12.767 50 2 39.233 2 26S12.767 2 26 2s24 10.767 24 24-10.767 24-24
24zm9.707-33.707a1 1 0 0 0-1.414 0L26 24.586l-8.293-8.293a1 1 0 0 0-1.414 1.414L24.586 26l-8.293 8.293a1 1 0 0 0 0 1.414c.195.195.451.293.707.293s.512-.098.707
-.293L26 27.414l8.293 8.293c.195.195.451.293.707.293s.512-.098.707-.293a1 1 0 0 0 0-1.414L27.414 26l8.293-8.293a1 1 0 0 0 0-1.414z"/>
</svg>
`;if(type==='success'){svgTemplate=`
<svg class="alert-img" xmlns="http://www.w3.org/2000/svg" fill="#fff" viewBox="0 0 52 52" xmlns:v="https://vecta.io/nano">
<path d="M26 0C11.664 0 0 11.663 0 26s11.664 26 26 26 26-11.663 26-26S40.336 0 26 0zm0 50C12.767 50 2 39.233 2 26S12.767 2 26 2s24 10.767 24 24-10.767 24-24
24zm12.252-34.664l-15.369 17.29-9.259-7.407a1 1 0 0 0-1.249 1.562l10 8a1 1 0 0 0 1.373-.117l16-18a1 1 0 1 0-1.496-1.328z"/>
</svg>
`;}
if(type==='info'){svgTemplate=`
<svg class="alert-img" xmlns="http://www.w3.org/2000/svg" fill="#fff" viewBox="0 0 64 64" xmlns:v="https://vecta.io/nano">
<path d="M38.535 47.606h-4.08V28.447a1 1 0 0 0-1-1h-4.52a1 1 0 1 0 0 2h3.52v18.159h-5.122a1 1 0 1 0 0 2h11.202a1 1 0 1 0 0-2z"/>
<circle cx="32" cy="18" r="3"/><path d="M32 0C14.327 0 0 14.327 0 32s14.327 32 32 32 32-14.327 32-32S49.673 0 32 0zm0 62C15.458 62 2 48.542 2 32S15.458 2 32 2s30 13.458 30 30-13.458 30-30 30z"/>
</svg>
`;}
const template=`
<div class="alert-wrapper">
  <div class="alert-frame">
    <div class="alert-header ${type}-bg">
      <span class="${closeStyleTemplate}">X</span>
      ${svgTemplate}
    </div>
    <div class="alert-body">
      <span class="alert-title">${title}</span>
      <span class="alert-message">${message}</span>
      ${btnTemplate}
    </div>
  </div>
</div>
`;body.insertAdjacentHTML('afterend',template);const alertWrapper=document.querySelector('.alert-wrapper');const alertFrame=document.querySelector('.alert-frame');const alertClose=document.querySelector(`.${closeStyleTemplate}`);function resolveIt(){alertWrapper.remove();resolve();}
function confirmIt(){alertWrapper.remove();resolve('confirm');}
function stopProp(e){e.stopPropagation();}
if(type==='question'){const confirmButton=document.querySelector('.confirm-button');const cancelButton=document.querySelector('.cancel-button');confirmButton.addEventListener('click',confirmIt);cancelButton.addEventListener('click',resolveIt);}else{const alertButton=document.querySelector('.alert-button');alertButton.addEventListener('click',resolveIt);}
alertClose.addEventListener('click',resolveIt);alertWrapper.addEventListener('click',resolveIt);alertFrame.addEventListener('click',stopProp);});}
function autocomplete(inp,arr){let currentFocus;function handler(e){let b;const val=this.value;closeAllLists();currentFocus=-1;const a=document.createElement('DIV');a.setAttribute('id',this.id+'autocomplete-list');a.setAttribute('class','autocomplete-items');this.parentNode.appendChild(a);for(let i=0;i<arr.length;i++){if(arr[i].substr(0,val.length).toUpperCase()==val.toUpperCase()){b=document.createElement('DIV');b.innerHTML='<strong>'+arr[i].substr(0,val.length)+'</strong>';b.innerHTML+=arr[i].substr(val.length);b.innerHTML+='<input type="hidden" value="'+arr[i]+'">';b.addEventListener('click',((arg)=>(e)=>{inp.value=arg.getElementsByTagName('input')[0].value;closeAllLists();})(b));a.appendChild(b);}}}
inp.addEventListener('input',handler);inp.addEventListener('click',handler);inp.addEventListener('keydown',(e)=>{let x=_(this.id+'autocomplete-list');if(x)x=x.getElementsByTagName('div');if(e.keyCode==40){currentFocus++;addActive(x);}else if(e.keyCode==38){currentFocus--;addActive(x);}else if(e.keyCode==13){e.preventDefault();if(currentFocus>-1){if(x)x[currentFocus].click();}}});function addActive(x){if(!x)return false;removeActive(x);if(currentFocus>=x.length)currentFocus=0;if(currentFocus<0)currentFocus=(x.length-1);x[currentFocus].classList.add('autocomplete-active');}
function removeActive(x){for(let i=0;i<x.length;i++){x[i].classList.remove('autocomplete-active');}}
function closeAllLists(elmnt){const x=document.getElementsByClassName('autocomplete-items');for(let i=0;i<x.length;i++){if(elmnt!=x[i]&&elmnt!=inp){x[i].parentNode.removeChild(x[i]);}}}
document.addEventListener('click',(e)=>{closeAllLists(e.target);});}
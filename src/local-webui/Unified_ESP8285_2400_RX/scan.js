document.addEventListener('DOMContentLoaded',init,false);let colorTimer=undefined;let colorUpdated=false;let storedModelId=255;let buttonActions=[];let modeSelectionInit=true;let originalUID=undefined;let originalUIDType=undefined;function _(el){return document.getElementById(el);}
function getPwmFormData(){let ch=0;let inField;const outData=[];while(inField=_(`pwm_${ch}_ch`)){const inChannel=inField.value;const mode=_(`pwm_${ch}_mode`).value;const invert=_(`pwm_${ch}_inv`).checked?1:0;const narrow=_(`pwm_${ch}_nar`).checked?1:0;const failsafeField=_(`pwm_${ch}_fs`);const failsafeModeField=_(`pwm_${ch}_fsmode`);let failsafe=failsafeField.value;if(failsafe>2011)failsafe=2011;if(failsafe<988)failsafe=988;failsafeField.value=failsafe;let failsafeMode=failsafeModeField.value;const raw=(narrow<<19)|(mode<<15)|(invert<<14)|(inChannel<<10)|(failsafeMode<<20)|(failsafe-988);outData.push(raw);++ch;}
return outData;}
function enumSelectGenerate(id,val,arOptions){const retVal=`<div class="mui-select compact"><select id="${id}" class="pwmitm">`+
arOptions.map((item,idx)=>{if(item)return`<option value="${idx}"${(idx === val) ? ' selected' : ''} ${item === 'Disabled' ? 'disabled' : ''}>${item}</option>`;return'';}).join('')+'</select></div>';return retVal;}
function generateFeatureBadges(features){let str='';if(!!(features&1))str+=`<span style="color: #696969; background-color: #a8dcfa" class="badge">TX</span>`;else if(!!(features&2))str+=`<span style="color: #696969; background-color: #d2faa8" class="badge">RX</span>`;if((features&12)===12)str+=`<span style="color: #696969; background-color: #fab4a8" class="badge">I2C</span>`;else if(!!(features&4))str+=`<span style="color: #696969; background-color: #fab4a8" class="badge">SCL</span>`;else if(!!(features&8))str+=`<span style="color: #696969; background-color: #fab4a8" class="badge">SDA</span>`;if((features&96)===96)str+=`<span style="color: #696969; background-color: #36b5ff" class="badge">Serial2</span>`;else if(!!(features&32))str+=`<span style="color: #696969; background-color: #36b5ff" class="badge">RX2</span>`;else if(!!(features&64))str+=`<span style="color: #696969; background-color: #36b5ff" class="badge">TX2</span>`;return str;}
function updatePwmSettings(arPwm){if(arPwm===undefined){if(_('model_tab'))_('model_tab').style.display='none';return;}
var pinRxIndex=undefined;var pinTxIndex=undefined;var pinModes=[]
const htmlFields=['<div class="mui-panel pwmpnl"><table class="pwmtbl mui-table"><tr><th class="fixed-column">Output</th><th class="mui--text-center fixed-column">Features</th><th>Mode</th><th>Input</th><th class="mui--text-center fixed-column">Invert?</th><th class="mui--text-center fixed-column">750us?</th><th class="mui--text-center fixed-column pwmitm">Failsafe Mode</th><th class="mui--text-center fixed-column pwmitm">Failsafe Pos</th></tr>'];arPwm.forEach((item,index)=>{const failsafe=(item.config&1023)+988;const failsafeMode=(item.config>>20)&3;const ch=(item.config>>10)&15;const inv=(item.config>>14)&1;const mode=(item.config>>15)&15;const narrow=(item.config>>19)&1;const features=item.features;const modes=['50Hz','60Hz','100Hz','160Hz','333Hz','400Hz','10KHzDuty','On/Off'];if(features&16){modes.push('DShot');}else{modes.push(undefined);}
if(features&1){modes.push('Serial TX');modes.push(undefined);modes.push(undefined);modes.push(undefined);pinRxIndex=index;}else if(features&2){modes.push('Serial RX');modes.push(undefined);modes.push(undefined);modes.push(undefined);pinTxIndex=index;}else{modes.push(undefined);if(features&4){modes.push('I2C SCL');}else{modes.push(undefined);}
if(features&8){modes.push('I2C SDA');}else{modes.push(undefined);}
modes.push(undefined);}
if(features&32){modes.push('Serial2 RX');}else{modes.push(undefined);}
if(features&64){modes.push('Serial2 TX');}else{modes.push(undefined);}
const modeSelect=enumSelectGenerate(`pwm_${index}_mode`,mode,modes);const inputSelect=enumSelectGenerate(`pwm_${index}_ch`,ch,['ch1','ch2','ch3','ch4','ch5 (AUX1)','ch6 (AUX2)','ch7 (AUX3)','ch8 (AUX4)','ch9 (AUX5)','ch10 (AUX6)','ch11 (AUX7)','ch12 (AUX8)','ch13 (AUX9)','ch14 (AUX10)','ch15 (AUX11)','ch16 (AUX12)']);const failsafeModeSelect=enumSelectGenerate(`pwm_${index}_fsmode`,failsafeMode,['Set Position','No Pulses','Last Position']);htmlFields.push(`<tr><td class="mui--text-center mui--text-title">${index + 1}</td>
            <td>${generateFeatureBadges(features)}</td>
            <td>${modeSelect}</td>
            <td>${inputSelect}</td>
            <td><div class="mui-checkbox mui--text-center"><input type="checkbox" id="pwm_${index}_inv"${(inv) ? ' checked' : ''}></div></td>
            <td><div class="mui-checkbox mui--text-center"><input type="checkbox" id="pwm_${index}_nar"${(narrow) ? ' checked' : ''}></div></td>
            <td>${failsafeModeSelect}</td>
            <td><div class="mui-textfield compact"><input id="pwm_${index}_fs" value="${failsafe}" size="6" class="pwmitm" /></div></td></tr>`);pinModes[index]=mode;});htmlFields.push('</table></div>');const grp=document.createElement('DIV');grp.setAttribute('class','group');grp.innerHTML=htmlFields.join('');_('pwm').appendChild(grp);const setDisabled=(index,onoff)=>{_(`pwm_${index}_ch`).disabled=onoff;_(`pwm_${index}_inv`).disabled=onoff;_(`pwm_${index}_nar`).disabled=onoff;_(`pwm_${index}_fs`).disabled=onoff;_(`pwm_${index}_fsmode`).disabled=onoff;}
arPwm.forEach((item,index)=>{const pinMode=_(`pwm_${index}_mode`)
pinMode.onchange=()=>{setDisabled(index,pinMode.value>9);const updateOthers=(value,enable)=>{if(value>9){arPwm.forEach((item,other)=>{if(other!=index){document.querySelectorAll(`#pwm_${other}_mode option`).forEach(opt=>{if(opt.value==value){if(modeSelectionInit)
opt.disabled=true;else
opt.disabled=enable;}});}})}}
updateOthers(pinMode.value,true);updateOthers(pinModes[index],false);pinModes[index]=pinMode.value;_('serial1-config').style.display='none';if(pinMode.value==14)
_('serial1-config').style.display='block';}
pinMode.onchange();const failsafeMode=_(`pwm_${index}_fsmode`);failsafeMode.onchange=()=>{const failsafeField=_(`pwm_${index}_fs`);if(failsafeMode.value==0){failsafeField.disabled=false;failsafeField.style.display='block';}
else{failsafeField.disabled=true;failsafeField.style.display='none';}};failsafeMode.onchange();});modeSelectionInit=false;if(pinRxIndex!==undefined&&pinTxIndex!==undefined){const pinRxMode=_(`pwm_${pinRxIndex}_mode`);const pinTxMode=_(`pwm_${pinTxIndex}_mode`);pinRxMode.onchange=()=>{if(pinRxMode.value==9){pinTxMode.value=9;setDisabled(pinRxIndex,true);setDisabled(pinTxIndex,true);pinTxMode.disabled=true;_('serial-config').style.display='block';_('baud-config').style.display='block';}
else{pinTxMode.value=0;setDisabled(pinRxIndex,false);setDisabled(pinTxIndex,false);pinTxMode.disabled=false;_('serial-config').style.display='none';_('baud-config').style.display='none';}}
pinTxMode.onchange=()=>{if(pinTxMode.value==9){pinRxMode.value=9;setDisabled(pinRxIndex,true);setDisabled(pinTxIndex,true);pinTxMode.disabled=true;_('serial-config').style.display='block';_('baud-config').style.display='block';}}
const pinTx=pinTxMode.value;pinRxMode.onchange();if(pinRxMode.value!=9)pinTxMode.value=pinTx;}}
function init(){_('nt0').onclick=()=>_('credentials').style.display='block';_('nt1').onclick=()=>_('credentials').style.display='block';_('nt2').onclick=()=>_('credentials').style.display='none';_('nt3').onclick=()=>_('credentials').style.display='none';_('model-match').onclick=()=>{if(_('model-match').checked){_('modelNum').style.display='block';if(storedModelId===255){_('modelid').value='';}else{_('modelid').value=storedModelId;}}else{_('modelNum').style.display='none';_('modelid').value='255';}};mui.tabs.activate('pane-justified-3');initFiledrag();initOptions();}
function updateUIDType(uidtype){let bg='';let fg='white';let desc='';if(!uidtype||uidtype.startsWith('Not set'))
{bg='#D50000';uidtype='Not set';desc='Using autogenerated binding UID';}
else if(uidtype==='Flashed')
{bg='#1976D2';desc='The binding UID was generated from a binding phrase set at flash time';}
else if(uidtype==='Overridden')
{bg='#689F38';fg='black';desc='The binding UID has been generated from a binding phrase previously entered into the "binding phrase" field above';}
else if(uidtype==='Modified')
{bg='#7c00d5';desc='The binding UID has been modified, but not yet saved';}
else if(uidtype==='Volatile')
{bg='#FFA000';desc='The binding UID will be cleared on boot';}
else if(uidtype==='Loaned')
{bg='#FFA000';desc='This receiver is on loan and can be returned using Lua or three-plug';}
else
{if(_('uid').value.endsWith('0,0,0,0'))
{bg='#FFA000';uidtype='Not bound';desc='This receiver is unbound and will boot to binding mode';}
else
{bg='#1976D2';uidtype='Bound';desc='This receiver is bound and will boot waiting for connection';}}
_('uid-type').style.backgroundColor=bg;_('uid-type').style.color=fg;_('uid-type').textContent=uidtype;_('uid-text').textContent=desc;}
function updateConfig(data,options){if(data.product_name)_('product_name').textContent=data.product_name;if(data.reg_domain)_('reg_domain').textContent=data.reg_domain;if(data.uid){_('uid').value=data.uid.toString();originalUID=data.uid;}
originalUIDType=data.uidtype;updateUIDType(data.uidtype);if(data.mode==='STA'){_('stamode').style.display='block';_('ssid').textContent=data.ssid;}else{_('apmode').style.display='block';}
if(data.hasOwnProperty('modelid')&&data.modelid!==255){_('modelNum').style.display='block';_('model-match').checked=true;storedModelId=data.modelid;}else{_('modelNum').style.display='none';_('model-match').checked=false;storedModelId=255;}
_('modelid').value=storedModelId;_('force-tlm').checked=data.hasOwnProperty('force-tlm')&&data['force-tlm'];_('serial-protocol').onchange=()=>{const proto=Number(_('serial-protocol').value);if(_('is-airport').checked){_('rcvr-uart-baud').disabled=false;_('rcvr-uart-baud').value=options['rcvr-uart-baud'];_('serial-config').style.display='none';_('sbus-config').style.display='none';return;}
_('serial-config').style.display='block';if(proto===0||proto===1){_('rcvr-uart-baud').disabled=false;_('rcvr-uart-baud').value=options['rcvr-uart-baud'];_('sbus-config').style.display='none';}
else if(proto===2||proto===3||proto===5){_('rcvr-uart-baud').disabled=true;_('rcvr-uart-baud').value='100000';_('sbus-config').style.display='block';_('sbus-failsafe').value=data['sbus-failsafe'];}
else if(proto===4){_('rcvr-uart-baud').disabled=true;_('rcvr-uart-baud').value='115200';_('sbus-config').style.display='none';}
else if(proto===6){_('rcvr-uart-baud').disabled=true;_('rcvr-uart-baud').value='19200';_('sbus-config').style.display='none';}}
_('serial1-protocol').onchange=()=>{if(_('is-airport').checked){_('rcvr-uart-baud').disabled=false;_('rcvr-uart-baud').value=options['rcvr-uart-baud'];_('serial1-config').style.display='none';_('sbus-config').style.display='none';return;}}
updatePwmSettings(data.pwm);_('serial-protocol').value=data['serial-protocol'];_('serial-protocol').onchange();_('serial1-protocol').value=data['serial1-protocol'];_('serial1-protocol').onchange();_('is-airport').onchange=()=>{_('serial-protocol').onchange();_('serial1-protocol').onchange();}
_('is-airport').onchange;_('vbind').value=data.vbind;_('vbind').onchange=()=>{_('bindphrase').style.display=_('vbind').value==='1'?'none':'block';}
_('vbind').onchange();_('serial1-config').style.display='none';data.pwm?.forEach((item,index)=>{const _pinMode=_(`pwm_${index}_mode`)
if(_pinMode.value==14)
_('serial1-config').style.display='block';});}
function initOptions(){const xmlhttp=new XMLHttpRequest();xmlhttp.onreadystatechange=function(){if(this.readyState===4&&this.status===200){const data=JSON.parse(this.responseText);updateOptions(data['options']);updateConfig(data['config'],data['options']);initBindingPhraseGen();}};xmlhttp.open('GET','/config',true);xmlhttp.send();}
function getNetworks(){const xmlhttp=new XMLHttpRequest();xmlhttp.onload=function(){if(this.status===204){setTimeout(getNetworks,2000);}else{const data=JSON.parse(this.responseText);if(data.length>0){_('loader').style.display='none';autocomplete(_('network'),data);}}};xmlhttp.onerror=function(){setTimeout(getNetworks,2000);};xmlhttp.open('GET','networks.json',true);xmlhttp.send();}
_('network-tab').addEventListener('mui.tabs.showstart',getNetworks);function initFiledrag(){const fileselect=_('firmware_file');const filedrag=_('filedrag');fileselect.addEventListener('change',fileSelectHandler,false);const xhr=new XMLHttpRequest();if(xhr.upload){filedrag.addEventListener('dragover',fileDragHover,false);filedrag.addEventListener('dragleave',fileDragHover,false);filedrag.addEventListener('drop',fileSelectHandler,false);filedrag.style.display='block';}}
function fileDragHover(e){e.stopPropagation();e.preventDefault();if(e.target===_('filedrag'))e.target.className=(e.type==='dragover'?'hover':'');}
function fileSelectHandler(e){fileDragHover(e);const files=e.target.files||e.dataTransfer.files;const fileExt=files[0].name.split('.').pop();const expectedFileExt='gz';const expectedFileExtDesc='.bin.gz file. <br />Do NOT decompress/unzip/extract the file!';if(fileExt===expectedFileExt){uploadFile(files[0]);}else{cuteAlert({type:'error',title:'Incorrect File Format',message:'You selected the file &quot;'+files[0].name.toString()+'&quot;.<br />The firmware file must be a '+expectedFileExtDesc});}}
function uploadFile(file){_('upload_btn').disabled=true
try{const formdata=new FormData();formdata.append('upload',file,file.name);const ajax=new XMLHttpRequest();ajax.upload.addEventListener('progress',progressHandler,false);ajax.addEventListener('load',completeHandler,false);ajax.addEventListener('error',errorHandler,false);ajax.addEventListener('abort',abortHandler,false);ajax.open('POST','/update');ajax.setRequestHeader('X-FileSize',file.size);ajax.send(formdata);}
catch(e){_('upload_btn').disabled=false}}
function progressHandler(event){const percent=Math.round((event.loaded/event.total)*100);_('progressBar').value=percent;_('status').innerHTML=percent+'% uploaded... please wait';}
function completeHandler(event){_('status').innerHTML='';_('progressBar').value=0;_('upload_btn').disabled=false
const data=JSON.parse(event.target.responseText);if(data.status==='ok'){function showMessage(){cuteAlert({type:'success',title:'Update Succeeded',message:data.msg});}
let percent=0;const interval=setInterval(()=>{percent=percent+1;_('progressBar').value=percent;_('status').innerHTML=percent+'% flashed... please wait';if(percent===100){clearInterval(interval);_('status').innerHTML='';_('progressBar').value=0;showMessage();}},100);}else if(data.status==='mismatch'){cuteAlert({type:'question',title:'Targets Mismatch',message:data.msg,confirmText:'Flash anyway',cancelText:'Cancel'}).then((e)=>{const xmlhttp=new XMLHttpRequest();xmlhttp.onreadystatechange=function(){if(this.readyState===4){_('status').innerHTML='';_('progressBar').value=0;if(this.status===200){const data=JSON.parse(this.responseText);cuteAlert({type:'info',title:'Force Update',message:data.msg});}else{cuteAlert({type:'error',title:'Force Update',message:'An error occurred trying to force the update'});}}};xmlhttp.open('POST','/forceupdate',true);const data=new FormData();data.append('action',e);xmlhttp.send(data);});}else{cuteAlert({type:'error',title:'Update Failed',message:data.msg});}}
function errorHandler(event){_('status').innerHTML='';_('progressBar').value=0;_('upload_btn').disabled=false
cuteAlert({type:'error',title:'Update Failed',message:event.target.responseText});}
function abortHandler(event){_('status').innerHTML='';_('progressBar').value=0;_('upload_btn').disabled=false
cuteAlert({type:'info',title:'Update Aborted',message:event.target.responseText});}
function setupNetwork(event){if(_('nt0').checked){postWithFeedback('Set Home Network','An error occurred setting the home network','/sethome?save',function(){return new FormData(_('sethome'));},function(){_('wifi-ssid').value=_('network').value;_('wifi-password').value=_('password').value;})(event);}
if(_('nt1').checked){postWithFeedback('Connect To Network','An error occurred connecting to the network','/sethome',function(){return new FormData(_('sethome'));})(event);}
if(_('nt2').checked){postWithFeedback('Start Access Point','An error occurred starting the Access Point','/access',null)(event);}
if(_('nt3').checked){postWithFeedback('Forget Home Network','An error occurred forgetting the home network','/forget',null)(event);}}
_('reset-model').addEventListener('click',postWithFeedback('Reset Model Settings','An error occurred reseting model settings','/reset?model',null));_('reset-options').addEventListener('click',postWithFeedback('Reset Runtime Options','An error occurred reseting runtime options','/reset?options',null));_('sethome').addEventListener('submit',setupNetwork);_('connect').addEventListener('click',postWithFeedback('Connect to Home Network','An error occurred connecting to the Home network','/connect',null));if(_('config')){_('config').addEventListener('submit',postWithFeedback('Set Configuration','An error occurred updating the configuration','/config',(xmlhttp)=>{xmlhttp.setRequestHeader('Content-Type','application/json');return JSON.stringify({"pwm":getPwmFormData(),"serial-protocol":+_('serial-protocol').value,"serial1-protocol":+_('serial1-protocol').value,"sbus-failsafe":+_('sbus-failsafe').value,"modelid":+_('modelid').value,"force-tlm":+_('force-tlm').checked,"vbind":+_('vbind').value,"uid":_('uid').value.split(',').map(Number),});},()=>{originalUID=_('uid').value;originalUIDType='Bound';_('phrase').value='';updateUIDType(originalUIDType);}));}
function submitOptions(e){e.stopPropagation();e.preventDefault();const xhr=new XMLHttpRequest();xhr.open('POST','/options.json');xhr.setRequestHeader('Content-Type','application/json');const formElem=_('upload_options');const formObject=Object.fromEntries(new FormData(formElem));formElem.querySelectorAll('input[type=checkbox]:not(:checked)').forEach((k)=>formObject[k.name]=false);formObject['customised']=true;xhr.send(JSON.stringify(formObject,function(k,v){if(v==='')return undefined;if(_(k)){if(_(k).type==='color')return undefined;if(_(k).type==='checkbox')return v==='on';if(_(k).classList.contains('datatype-boolean'))return v==='true';if(_(k).classList.contains('array')){const arr=v.split(',').map((element)=>{return Number(element);});return arr.length===0?undefined:arr;}}
if(typeof v==='boolean')return v;if(v==='true')return true;if(v==='false')return false;return isNaN(v)?v:+v;}));xhr.onreadystatechange=function(){if(this.readyState===4){if(this.status===200){cuteAlert({type:'question',title:'Upload Succeeded',message:'Reboot to take effect',confirmText:'Reboot',cancelText:'Close'}).then((e)=>{if(e==='confirm'){const xhr=new XMLHttpRequest();xhr.open('POST','/reboot');xhr.setRequestHeader('Content-Type','application/json');xhr.onreadystatechange=function(){};xhr.send();}});}else{cuteAlert({type:'error',title:'Upload Failed',message:this.responseText});}}};}
_('submit-options').addEventListener('click',submitOptions);function updateOptions(data){for(const[key,value]of Object.entries(data)){if(key==='wifi-on-interval'&&value===-1)continue;if(_(key)){if(_(key).type==='checkbox'){_(key).checked=value;}else{if(Array.isArray(value))_(key).value=value.toString();else _(key).value=value;}
if(_(key).onchange)_(key).onchange();}}
if(data['wifi-ssid'])_('homenet').textContent=data['wifi-ssid'];else _('connect').style.display='none';if(data['customised'])_('reset-options').style.display='block';_('submit-options').disabled=false;}
md5=function(){const k=[];let i=0;for(;i<64;){k[i]=0|(Math.abs(Math.sin(++i))*4294967296);}
function calcMD5(str){let b;let c;let d;let j;const x=[];const str2=unescape(encodeURI(str));let a=str2.length;const h=[b=1732584193,c=-271733879,~b,~c];let i=0;for(;i<=a;)x[i>>2]|=(str2.charCodeAt(i)||128)<<8*(i++%4);x[str=(a+8>>6)*16+14]=a*8;i=0;for(;i<str;i+=16){a=h;j=0;for(;j<64;){a=[d=a[3],((b=a[1]|0)+
((d=((a[0]+
[b&(c=a[2])|~b&d,d&b|~d&c,b^c^d,c^(b|~d)][a=j>>4])+
(k[j]+
(x[[j,5*j+1,3*j+5,7*j][a]%16+i]|0))))<<(a=[7,12,17,22,5,9,14,20,4,11,16,23,6,10,15,21][4*a+j++%4])|d>>>32-a)),b,c];}
for(j=4;j;)h[--j]=h[j]+a[j];}
str=[];for(;j<32;)str.push(((h[j>>3]>>((1^j++&7)*4))&15)*16+((h[j>>3]>>((1^j++&7)*4))&15));return new Uint8Array(str);}
return calcMD5;}();function isValidUidByte(s){let f=parseFloat(s);return!isNaN(f)&&isFinite(s)&&Number.isInteger(f)&&f>=0&&f<256;}
function uidBytesFromText(text){if(/^[0-9, ]+$/.test(text))
{let asArray=text.split(',').filter(isValidUidByte).map(Number);if(asArray.length>=4&&asArray.length<=6)
{while(asArray.length<6)
asArray.unshift(0);return asArray;}}
const bindingPhraseFull=`-DMY_BINDING_PHRASE="${text}"`;const bindingPhraseHashed=md5(bindingPhraseFull);return bindingPhraseHashed.subarray(0,6);}
function initBindingPhraseGen(){const uid=_('uid');function setOutput(text){if(text.length===0){uid.value=originalUID.toString();updateUIDType(originalUIDType);}
else{uid.value=uidBytesFromText(text.trim());updateUIDType('Modified');}}
function updateValue(e){setOutput(e.target.value);}
_('phrase').addEventListener('input',updateValue);setOutput('');}
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
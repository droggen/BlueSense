/* This file is part of Natural Docs, which is Copyright © 2003-2013 Greg Valure.
 * Natural Docs is licensed under version 3 of the GNU Affero General Public
 * License (AGPL).  Refer to License.txt or www.naturaldocs.org for the
 * complete details.
 *
 * This file may be distributed with documentation files generated by Natural Docs.
 * Such documentation is not covered by Natural Docs' copyright and licensing,
 * and may have its own copyright and distribution terms as decided by its author.
 */

"use strict";var NDMenu=new function(){this.Start=function(){this.menuSections=[];this.firstUnusedMenuSection=0;var menuContainer=document.getElementById("NDMenu");var menuTabBar=document.createElement("div");menuTabBar.id="MTabBar";menuContainer.appendChild(menuTabBar);var menuContent=document.createElement("div");menuContent.id="MContent";menuContainer.appendChild(menuContent);NDCore.LoadJavaScript("menu/tabs.js","NDMenuTabsLoader");};this.OnLocationChange=function(oldLocation,newLocation){if(newLocation.type=="Home"&&this.tabs!=undefined&&this.tabs.length==1){this.GoToOffsets([0]);return;}if(oldLocation==undefined||oldLocation.type!=newLocation.type){this.UpdateTabs(newLocation.type);}if(oldLocation==undefined||oldLocation.type!=newLocation.type||oldLocation.path!=newLocation.path){this.Build(new NDMenuHashPath(newLocation.type,newLocation.path));}};this.GoToOffsets=function(offsets){if(this.tabs!=undefined){var newSelectedTab;if(offsets.length>=1){newSelectedTab=this.tabs[offsets[0]][0];if(newSelectedTab!=this.selectedTabType){this.UpdateTabs(newSelectedTab);}}}this.Build(new NDMenuOffsetPath(offsets));};this.Build=function(path){if(path!=undefined){this.pathBeingBuilt=path;}else if(this.pathBeingBuilt==undefined){return;}this.firstUnusedMenuSection=0;var newMenuContent=document.createElement("div");newMenuContent.id="MContent";var result;if(this.tabs!=undefined){result=this.BuildEntries(newMenuContent,this.pathBeingBuilt);}else{result={completed:false};}if(!result.completed){var htmlEntry=document.createElement("div");htmlEntry.className="MLoadingNotice";newMenuContent.appendChild(htmlEntry);}var oldMenuContent=document.getElementById("MContent");var menuContainer=oldMenuContent.parentNode;menuContainer.replaceChild(newMenuContent,oldMenuContent);if(NDCore.IsIE()&&NDCore.IEVersion()==10){setTimeout(function(){document.getElementById("NDMenu").style.zoom="100%";},0);}if(result.completed){if(result.selectedFile){if(result.selectedFile.offsetTop+result.selectedFile.offsetHeight>menuContainer.scrollTop+menuContainer.clientHeight){result.selectedFile.scrollIntoView(false);}else if(result.selectedFile.offsetTop<menuContainer.scrollTop){result.selectedFile.scrollIntoView(true);}}this.pathBeingBuilt=undefined;this.CleanUpMenuSections();}else if(result.needToLoad!=undefined){this.LoadMenuSection(result.needToLoad);}};this.BuildEntries=function(htmlMenu,path){var result={completed:false};var pathSoFar=[];var iterator=path.GetIterator();var navigationType;var selectedTab;for(;;){navigationType=iterator.NavigationType();if(navigationType== 9){result.needToLoad=iterator.needToLoad;return result;}else if(navigationType== 3||navigationType== -1){break;}else if(iterator.InTabs()){pathSoFar.push(iterator.CurrentContainerIndex());selectedTab=iterator.CurrentEntry();if(typeof(selectedTab[1])=="object"){for(var i=1;i<selectedTab[1].length-1;i++){var htmlEntry=document.createElement("div");htmlEntry.className="MEntry MFolder Parent Empty";htmlEntry.innerHTML=selectedTab[1][i];htmlMenu.appendChild(htmlEntry);}}var title;if(typeof(selectedTab[1])=="object"){title=selectedTab[1][selectedTab[1].length-1];}else if(this.tabs.length==1){title=selectedTab[1];}if(title!=undefined){if(navigationType== 2){var htmlEntry=document.createElement("div");htmlEntry.className="MEntry MFolder Selected";htmlEntry.innerHTML=title;htmlMenu.appendChild(htmlEntry);}else{var htmlEntry=document.createElement("a");htmlEntry.className="MEntry MFolder Parent";htmlEntry.setAttribute("href","javascript:NDMenu.GoToTab(\""+selectedTab[0]+"\")");htmlEntry.innerHTML=title;htmlMenu.appendChild(htmlEntry);}}iterator.Next();}else if(navigationType== 1||navigationType== 2){pathSoFar.push(iterator.CurrentContainerIndex());var currentEntry=iterator.CurrentEntry();var title;if(typeof(currentEntry[1])=="object"){for(var i=0;i<currentEntry[1].length-1;i++){var htmlEntry=document.createElement("div");htmlEntry.className="MEntry MFolder Parent Empty";htmlEntry.innerHTML=currentEntry[1][i];htmlMenu.appendChild(htmlEntry);}title=currentEntry[1][currentEntry[1].length-1];}else{title=currentEntry[1];}if(navigationType== 2){var htmlEntry=document.createElement("div");htmlEntry.className="MEntry MFolder Selected";htmlEntry.innerHTML=title;htmlMenu.appendChild(htmlEntry);}else{var htmlEntry=document.createElement("a");htmlEntry.className="MEntry MFolder Parent";htmlEntry.setAttribute("href","javascript:NDMenu.GoToOffsets(["+pathSoFar.join(",")+"])");htmlEntry.innerHTML=title;htmlMenu.appendChild(htmlEntry);}iterator.Next();}else{throw"Unexpected navigation type "+navigationType;}}var selectedFolder=iterator.CurrentContainer();var selectedFolderHashPath=iterator.CurrentContainerHashPath();var selectedFileIndex=iterator.CurrentContainerIndex();if(iterator.InTabs()){for(var i=0;i<this.tabs.length;i++){var htmlEntry=document.createElement("a");htmlEntry.className="MEntry MTabAsFolder";htmlEntry.id="M"+this.tabs[i][0]+"Tab";htmlEntry.setAttribute("href","javascript:NDMenu.GoToTab(\""+this.tabs[i][0]+"\")");var tabTitle=this.tabs[i][1];if(typeof(tabTitle)=="object"){tabTitle=tabTitle[0];}htmlEntry.innerHTML="<span class=\"MFolderIcon\"></span>"+"<span class=\"MTabIcon\"></span>"+"<span class=\"MTabTitle\">"+tabTitle+"</span>";htmlMenu.appendChild(htmlEntry);}}else{for(var i=0;i<selectedFolder.length;i++){var member=selectedFolder[i];if(member[0]== 1){if(i==selectedFileIndex){var htmlEntry=document.createElement("div");htmlEntry.className="MEntry MFile Selected";htmlEntry.innerHTML=member[1];htmlMenu.appendChild(htmlEntry);result.selectedFile=htmlEntry;}else{var hashPath=selectedFolderHashPath;if(member[2]==undefined){hashPath+=member[1];}else{hashPath+=member[2];}var htmlEntry=document.createElement("a");htmlEntry.className="MEntry MFile";htmlEntry.setAttribute("href","#"+hashPath);htmlEntry.innerHTML=member[1];htmlMenu.appendChild(htmlEntry);}}else{var title="<span class=\"MFolderIcon\"></span>";if(typeof(member[1])=="object"){title+=member[1][0];}else{title+=member[1];}var targetPath=(pathSoFar.length==0?i:pathSoFar.join(",")+","+i);var htmlEntry=document.createElement("a");htmlEntry.className="MEntry MFolder Child";htmlEntry.setAttribute("href","javascript:NDMenu.GoToOffsets(["+targetPath+"])");htmlEntry.innerHTML=title;htmlMenu.appendChild(htmlEntry);}}}result.completed=true;if(selectedTab!=undefined){selectedTab[6]=pathSoFar;}return result;};this.MenuSection=function(file){for(var i=0;i<this.menuSections.length;i++){if(this.menuSections[i].file==file){var section=this.menuSections[i];if(i>=this.firstUnusedMenuSection){if(i>this.firstUnusedMenuSection){this.menuSections.splice(i,1);this.menuSections.splice(this.firstUnusedMenuSection,0,section);}this.firstUnusedMenuSection++;}if(section.ready==true){return section.contents;}else{return undefined;}}}return undefined;};this.LoadMenuSection=function(file){for(var i=0;i<this.menuSections.length;i++){if(this.menuSections[i].file==file){return;}}var entry={file:file,contents:undefined,ready:false,domLoaderID:"NDMenuLoader_"+file.replace(/[^a-z0-9]/gi,"_")};this.menuSections.push(entry);NDCore.LoadJavaScript("menu/"+file,entry.domLoaderID);};this.OnSectionLoaded=function(file,contents){for(var i=0;i<this.menuSections.length;i++){if(this.menuSections[i].file==file){this.menuSections[i].contents=contents;this.menuSections[i].ready=true;NDCore.RemoveScriptElement(this.menuSections[i].domLoaderID);break;}}this.Build();};this.CleanUpMenuSections=function(){if(this.menuSections.length> 10){for(var i=this.menuSections.length-1;i>=this.firstUnusedMenuSection&&this.menuSections.length> 10;i--){if(this.menuSections[i].ready==false){break;}this.menuSections.pop();}}};this.OnTabsLoaded=function(tabs){this.tabs=tabs;NDCore.RemoveScriptElement("NDMenuTabsLoader");var tabBar=document.getElementById("MTabBar");for(var i=0;i<tabs.length;i++){var tab=document.createElement("a");tab.id="M"+tabs[i][0]+"Tab";tab.className="MTab Wide";tab.setAttribute("href","javascript:NDMenu.GoToTab(\""+tabs[i][0]+"\");");var tabTitle=tabs[i][1];if(typeof(tabTitle)=="object"){tabTitle=tabTitle[0];}tab.innerHTML="<span class=\"MTabIcon\"></span><span class=\"MTabTitle\">"+tabTitle+"</span>";tab.style.position="absolute";tab.style.visibility="hidden";tabBar.appendChild(tab);tabs[i][4]=tab.offsetWidth;tab.className="MTab Narrow";tabs[i][5]=tab.offsetWidth;if(tabs[i][0]==this.selectedTabType){tab.className+=" Selected";}}this.ResizeTabs();if(this.ShouldTabsShow()==false){tabBar.style.display="none";}for(var i=0;i<tabs.length;i++){var tab=this.GetTabElement(tabs[i][0]);tab.style.position="static";tab.style.visibility="visible";}if((this.selectedTabType==undefined||this.selectedTabType=="Home")&&this.tabs.length==1){this.GoToOffsets([0]);}else{this.Build();}};this.UpdateTabs=function(newTabType){if(newTabType==this.selectedTabType){return;}if(this.tabs!=undefined){if(this.selectedTabType!=undefined){var tab=this.GetTabElement(this.selectedTabType);if(tab!=undefined){NDCore.RemoveClass(tab,"Selected");}}if(newTabType!=undefined){var tab=this.GetTabElement(newTabType);if(tab!=undefined){NDCore.AddClass(tab,"Selected");}}}var wasShowing=this.ShouldTabsShow();this.selectedTabType=newTabType;var shouldShow=this.ShouldTabsShow();if(wasShowing!=shouldShow){var tabBar=document.getElementById("MTabBar");if(shouldShow){tabBar.style.display="block";}else{tabBar.style.display="none";}}this.ResizeTabs();};this.ResizeTabs=function(){if(this.ShouldTabsShow()==false){return;}var menu=document.getElementById("NDMenu");var menuWidth=menu.clientWidth;var allWideWidth=0;var selectedWideWidth=0;for(var i=0;i<this.tabs.length;i++){allWideWidth+=this.tabs[i][4];if(this.tabs[i][0]==this.selectedTabType){selectedWideWidth+=this.tabs[i][4];}else{selectedWideWidth+=this.tabs[i][5];}}for(var i=0;i<this.tabs.length;i++){var makeWide;if(allWideWidth<menuWidth){makeWide=true;}else if(selectedWideWidth<menuWidth){makeWide=(this.tabs[i][0]==this.selectedTabType);}else{makeWide=false;}var tab=this.GetTabElement(this.tabs[i][0]);if(makeWide){if(NDCore.HasClass(tab,"Narrow")){NDCore.RemoveClass(tab,"Narrow");NDCore.AddClass(tab,"Wide");}}else{if(NDCore.HasClass(tab,"Wide")){NDCore.RemoveClass(tab,"Wide");NDCore.AddClass(tab,"Narrow");}}}};this.ShouldTabsShow=function(){return(this.tabs!==undefined&&this.tabs.length>1&&this.selectedTabType!=undefined&&this.selectedTabType!="Home");};this.OnUpdateLayout=function(){this.ResizeTabs();};this.GoToTab=function(newTabType){var tabIndex;for(var i=0;i<this.tabs.length;i++){if(this.tabs[i][0]==newTabType){tabIndex=i;break;}}if(this.selectedTabType==newTabType){this.GoToOffsets([tabIndex]);}else if(newTabType==NDFramePage.currentLocation.type){this.UpdateTabs(newTabType);this.Build(new NDMenuHashPath(NDFramePage.currentLocation.type,NDFramePage.currentLocation.path));}else if(this.tabs[tabIndex][6]!=undefined){this.GoToOffsets(this.tabs[tabIndex][6]);}else{this.GoToOffsets([tabIndex]);}};this.GetTabElement=function(type){return document.getElementById("M"+type+"Tab");};};function NDMenuOffsetPath(offsets){this.GetIterator=function(){return new NDMenuOffsetPathIterator(this);};this.IsEmpty=function(){return(this.path.length==0);};if(offsets==undefined){this.path=[];}else{this.path=offsets;}};function NDMenuOffsetPathIterator(pathObject){this.Constructor=function(pathObject){this.pathObject=pathObject;this.pathIndex=0;this.currentContainer=NDMenu.tabs;if(NDMenu.tabs==undefined){this.needToLoad="tabs.js";}};this.Next=function(){if(this.pathIndex>=this.pathObject.path.length){return;}else if(this.currentContainer==undefined){this.pathIndex++;}else{var currentEntry=this.currentContainer[this.pathObject.path[this.pathIndex]];this.currentContainer=undefined;this.currentContainerHashPath=undefined;this.needToLoad=undefined;if(this.pathIndex==0){this.currentContainerHashPath=currentEntry[2];this.needToLoad=currentEntry[3];}else if(currentEntry[0]== 2){this.currentContainerHashPath=currentEntry[2];if(typeof currentEntry[3]=="string"){this.needToLoad=currentEntry[3];}else{this.currentContainer=currentEntry[3];}}this.pathIndex++;if(this.needToLoad!=undefined){this.currentContainer=NDMenu.MenuSection(this.needToLoad);if(this.currentContainer!=undefined){this.needToLoad=undefined;}}}};this.NavigationType=function(){if(this.currentContainer==undefined){return 9;}if(this.pathIndex>=this.pathObject.path.length){return -1;}var currentEntry=this.currentContainer[this.pathObject.path[this.pathIndex]];if(this.InTabs()==false&&currentEntry[0]== 1){return 3;}if(this.pathIndex==this.pathObject.path.length-1){return 2;}if(this.pathIndex+2<=this.pathObject.path.length-1){return 1;}var lookahead=this.Duplicate();lookahead.Next();if(lookahead.NavigationType()== 9){this.needToLoad=lookahead.NeedToLoad();return 9;}else if(lookahead.CurrentEntry()[0]== 2){return 1;}else{return 2;}};this.Duplicate=function(){var newObject=new NDMenuOffsetPathIterator(this.pathObject);newObject.pathIndex=this.pathIndex;newObject.currentContainer=this.currentContainer;newObject.needToLoad=this.needToLoad;return newObject;};this.CurrentEntry=function(){if(this.currentContainer!=undefined&&this.pathIndex<this.pathObject.path.length){return this.currentContainer[this.pathObject.path[this.pathIndex]];}else{return undefined;}};this.CurrentContainer=function(){return this.currentContainer;};this.CurrentContainerIndex=function(){if(this.pathIndex<this.pathObject.path.length){return this.pathObject.path[this.pathIndex];}else{return undefined;}};this.CurrentContainerHashPath=function(){return this.currentContainerHashPath;};this.InTabs=function(){return(this.pathIndex==0);};this.NeedToLoad=function(){if(this.currentContainer==undefined){return this.needToLoad;}else{return undefined;}};this.Constructor(pathObject);};function NDMenuHashPath(type,hashPath){this.GetIterator=function(){return new NDMenuOffsetPathIterator(this.MakeOffsetPath());};this.IsEmpty=function(){return(this.type==undefined||this.type=="Home");};this.MakeOffsetPath=function(){var offsets=[];if(this.type===undefined){return new NDMenuOffsetPath(offsets);}if(NDMenu.tabs===undefined){offsets.push(-1);return new NDMenuOffsetPath(offsets);}var tab;for(var i=0;i<NDMenu.tabs.length;i++){if(NDMenu.tabs[i][0]==this.type){tab=NDMenu.tabs[i];offsets.push(i);break;}}if(tab===undefined||this.hashPathString==""||this.hashPathString===undefined){return new NDMenuOffsetPath(offsets);}if(tab[2]!=undefined&&this.hashPathString.StartsWith(tab[2])==false){return new NDMenuOffsetPath(offsets);}var container=NDMenu.MenuSection(tab[3]);var containerHashPath=tab[2];if(container===undefined){offsets.push(-1);return new NDMenuOffsetPath(offsets);}do{var continueSearch=false;for(var i=0;i<container.length;i++){var member=container[i];if(member[0]== 1){var memberHashPath=containerHashPath;if(member[2]!==undefined){memberHashPath+=member[2];}else{memberHashPath+=member[1];}if(memberHashPath==this.hashPathString){offsets.push(i);return new NDMenuOffsetPath(offsets);}}else{if(this.hashPathString==member[2]){offsets.push(i);return new NDMenuOffsetPath(offsets);}else if(this.hashPathString.StartsWith(member[2])){offsets.push(i);containerHashPath=member[2];continueSearch=true;if(typeof member[3]=="string"){container=NDMenu.MenuSection(member[3]);if(container===undefined){offsets.push(-1);return new NDMenuOffsetPath(offsets);}}else{container=member[3];}break;}}}}while(continueSearch==true);return new NDMenuOffsetPath(offsets);};this.type=type;this.hashPathString=hashPath;};
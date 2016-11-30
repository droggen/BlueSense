NDSummary.OnToolTipsLoaded("File:spi.c",{15:"<div class=\"NDToolTip TFile LC\"><div class=\"TTSummary\">SPI functions.</div></div>",17:"<div class=\"NDToolTip TFunction LC\"><div id=\"NDPrototype17\" class=\"NDPrototype WideForm CStyle\"><table><tr><td class=\"PBeforeParameters\"><span class=\"SHKeyword\">void</span> spi_init(</td><td class=\"PParametersParentCell\"><table class=\"PParameters\"><tr><td class=\"PModifierQualifier first\"><span class=\"SHKeyword\">unsigned</span>&nbsp;</td><td class=\"PType\"><span class=\"SHKeyword\">char</span>&nbsp;</td><td class=\"PName last\">spidiv</td></tr></table></td><td class=\"PAfterParameters\">)</td></tr></table></div><div class=\"TTSummary\">Initialise SPI as Master in mode 0.</div></div>",18:"<div class=\"NDToolTip TFunction LC\"><div id=\"NDPrototype18\" class=\"NDPrototype WideForm CStyle\"><table><tr><td class=\"PBeforeParameters\"><span class=\"SHKeyword\">unsigned char</span> spi_rw(</td><td class=\"PParametersParentCell\"><table class=\"PParameters\"><tr><td class=\"PModifierQualifier first\"><span class=\"SHKeyword\">unsigned</span>&nbsp;</td><td class=\"PType\"><span class=\"SHKeyword\">char</span>&nbsp;</td><td class=\"PName last\">data</td></tr></table></td><td class=\"PAfterParameters\">)</td></tr></table></div><div class=\"TTSummary\">Exchange one byte with an SPI slave: sends one byte to the slave and receive one byte from it.</div></div>",19:"<div class=\"NDToolTip TFunction LC\"><div id=\"NDPrototype19\" class=\"NDPrototype WideForm CStyle\"><table><tr><td class=\"PBeforeParameters\"><span class=\"SHKeyword\">unsigned char</span> spi_rw_noselect(</td><td class=\"PParametersParentCell\"><table class=\"PParameters\"><tr><td class=\"PModifierQualifier first\"><span class=\"SHKeyword\">unsigned</span>&nbsp;</td><td class=\"PType\"><span class=\"SHKeyword\">char</span>&nbsp;</td><td class=\"PName last\">data</td></tr></table></td><td class=\"PAfterParameters\">)</td></tr></table></div><div class=\"TTSummary\">Exchange one byte with an SPI slave: sends one byte to the slave and receive one byte from it.</div></div>",20:"<div class=\"NDToolTip TFunction LC\"><div id=\"NDPrototype20\" class=\"NDPrototype WideForm CStyle\"><table><tr><td class=\"PBeforeParameters\"><span class=\"SHKeyword\">void</span> spi_rwn(</td><td class=\"PParametersParentCell\"><table class=\"PParameters\"><tr><td class=\"PModifierQualifier first\"><span class=\"SHKeyword\">unsigned</span>&nbsp;</td><td class=\"PType\"><span class=\"SHKeyword\">char</span>&nbsp;</td><td class=\"PNamePrefix\">*</td><td class=\"PName last\">ptr,</td></tr><tr><td class=\"PModifierQualifier first\"><span class=\"SHKeyword\">unsigned</span>&nbsp;</td><td class=\"PType\"><span class=\"SHKeyword\">char</span>&nbsp;</td><td></td><td class=\"PName last\">n</td></tr></table></td><td class=\"PAfterParameters\">)</td></tr></table></div><div class=\"TTSummary\">Exchanges n bytes with the SPI slave. Sends a n-byte buffer to the slave and receive n bytes from it.</div></div>",21:"<div class=\"NDToolTip TFunction LC\"><div id=\"NDPrototype21\" class=\"NDPrototype WideForm CStyle\"><table><tr><td class=\"PBeforeParameters\"><span class=\"SHKeyword\">void</span> spi_rwn_noselect(</td><td class=\"PParametersParentCell\"><table class=\"PParameters\"><tr><td class=\"PModifierQualifier first\"><span class=\"SHKeyword\">unsigned</span>&nbsp;</td><td class=\"PType\"><span class=\"SHKeyword\">char</span>&nbsp;</td><td class=\"PNamePrefix\">*</td><td class=\"PName last\">ptr,</td></tr><tr><td class=\"PModifierQualifier first\"><span class=\"SHKeyword\">unsigned</span>&nbsp;</td><td class=\"PType\"><span class=\"SHKeyword\">short</span>&nbsp;</td><td></td><td class=\"PName last\">n</td></tr></table></td><td class=\"PAfterParameters\">)</td></tr></table></div><div class=\"TTSummary\">Exchanges n bytes with the SPI slave. Sends a n-byte buffer to the slave and receive n bytes from it.</div></div>",22:"<div class=\"NDToolTip TFunction LC\"><div id=\"NDPrototype22\" class=\"NDPrototype WideForm CStyle\"><table><tr><td class=\"PBeforeParameters\"><span class=\"SHKeyword\">void</span> spi_wn_noselect(</td><td class=\"PParametersParentCell\"><table class=\"PParameters\"><tr><td class=\"PModifierQualifier first\"><span class=\"SHKeyword\">unsigned</span>&nbsp;</td><td class=\"PType\"><span class=\"SHKeyword\">char</span>&nbsp;</td><td class=\"PNamePrefix\">*</td><td class=\"PName last\">ptr,</td></tr><tr><td class=\"PModifierQualifier first\"><span class=\"SHKeyword\">unsigned</span>&nbsp;</td><td class=\"PType\"><span class=\"SHKeyword\">short</span>&nbsp;</td><td></td><td class=\"PName last\">n</td></tr></table></td><td class=\"PAfterParameters\">)</td></tr></table></div><div class=\"TTSummary\">Writes n bytes to an SPI slave without storing the value returned by the slave.</div></div>"});
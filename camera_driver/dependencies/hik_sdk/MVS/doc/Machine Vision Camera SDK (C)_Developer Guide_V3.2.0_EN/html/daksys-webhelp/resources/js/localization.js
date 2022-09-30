/*

DAKOTASYSTEMS Plugin
Copyright (c) 1998-2017 DAKOTASYSTEMS.  All rights reserved.

*/

/**
 * get translated messages
 */
function getLocalization(localizationKey) {
  var toReturn=localizationKey;
  if((localizationKey in localization)){
    toReturn=localization[localizationKey];
  }
  debug('getLocalization('+localizationKey+')='+toReturn);
  return toReturn;
}

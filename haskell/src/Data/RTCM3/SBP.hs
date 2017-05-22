{-# LANGUAGE LambdaCase        #-}
{-# LANGUAGE NoImplicitPrelude #-}

-- |
-- Module:      Data.RTCM3.SBP
-- Copyright:   Copyright (C) 2016 Swift Navigation, Inc.
-- License:     LGPL-3
-- Maintainer:  Swift Navigation <dev@swiftnav.com>
-- Stability:   experimental
-- Portability: portable
--
-- RTCMv3 to SBP Conversions.

module Data.RTCM3.SBP
  ( l1CSidCode
  , l2CMSidCode
  , l2PSidCode
  , sbpCMinM
  , q32Width
  , toWn
  , mjdEpoch
  , updateGpsTime
  , convert
  , newStore
  , validateIodcIode
  , gpsUriToUra
  ) where

import           BasicPrelude
import           Control.Lens
import           Control.Monad.Extra
import           Data.Bits
import qualified Data.HashMap.Strict  as M
import           Data.IORef
import           Data.List.Extra      hiding (concat, map)
import           Data.RTCM3
import           Data.RTCM3.SBP.Types
import           Data.Time
import           Data.Int ( Int8 )
import           Numeric.Lens ( multiplying )
import           Data.Word
import           SwiftNav.SBP


--------------------------------------------------------------------------------
-- SBP, GNSS, RTCM constant definitions


-- | Two centimeters
--
-- SBP pseudoranges are in units of 2cm, and there are 50 of those in a meter.
twoCM :: Double
twoCM = 0.02

sbpCMinM :: Double
sbpCMinM = 50

-- | Speed of light (meters/msec)
lightSpeedMMSEC :: Double
lightSpeedMMSEC = 299792.458

-- | Speed of light (meters/sec)
lightSpeedMS :: Double
lightSpeedMS = 299792458.0

-- | L1 GPS center frequency (Hz)
l1frequency :: Double
l1frequency = 1.57542e9

-- | L2 GPS center frequency (Hz)
l2frequency :: Double
l2frequency = 1.22760e9

-- | We only support PRNS 1 - 32
maxSats :: Word8
maxSats = 32

-- | Q32.8 carrier phase representation width
q32Width :: Double
q32Width = 256

-- | RTCM phase range resolution.
--
-- See DF018, pg. 3-19 of the RTCM3 spec
phaseRangeRes :: Double
phaseRangeRes = 0.0005

-- | SBP L1 GNSS signal value
--
-- See: https://github.com/swift-nav/libswiftnav/blob/master/include/libswiftnav/signal.h#L65
l1CSidCode :: Word8
l1CSidCode = 0

-- | SBP L2CM GNSS signal value
--
-- See: https://github.com/swift-nav/libswiftnav/blob/master/include/libswiftnav/signal.h#L65
l2CMSidCode :: Word8
l2CMSidCode = 1

-- | SBP L1 GLONASS signal value
--
-- See: https://github.com/swift-nav/libswiftnav/blob/master/include/libswiftnav/signal.h#L103
glonassL1CASidCode :: Word8
glonassL1CASidCode = 3

-- | SBP L2 GLONASS signal value
--
-- See: https://github.com/swift-nav/libswiftnav/blob/master/include/libswiftnav/signal.h#L104
glonassL2CASidCode :: Word8
glonassL2CASidCode = 4

-- | SBP L1P and L2P GNSS signal values
--
-- Note that libswiftnav currently does not support L1P or L2P observations,
-- just L1C and L2C. This is a stop gap definition so that we can properly
-- serialize out SBP observations with L1P and L2P observations from external
-- receivers.
l1PSidCode :: Word8
l1PSidCode = 5

l2PSidCode :: Word8
l2PSidCode = 6

-- | L2C code indicator value
--
-- See DF016, pg. 3-17 of the RTCM3 spec
codeIndicator_L2C :: Word8
codeIndicator_L2C = 0

-- | L2P code direct indicator value
--
-- See DF016, pg. 3-17 of the RTCM3 spec
codeIndicator_L2P :: Word8
codeIndicator_L2P = 1

-- | L2P code cross-correlated indicator value
--
-- See DF016, pg. 3-17 of the RTCM3 spec
codeIndicator_L2D :: Word8
codeIndicator_L2D = 2

-- | L2P code correlated indicator value
--
-- See DF016, pg. 3-17 of the RTCM3 spec
codeIndicator_L2W :: Word8
codeIndicator_L2W = 3

-- | Map L2 codes to SBP GnssSignal codes
--
l2codeToSBPSignalCode :: HashMap Word8 Word8
l2codeToSBPSignalCode = M.fromList
  [ (codeIndicator_L2C, l2CMSidCode)
  , (codeIndicator_L2P, l2PSidCode)
  , (codeIndicator_L2D, l2PSidCode)
  , (codeIndicator_L2W, l2PSidCode)
  ]

-- | Maximum number of packed observations to allow in a single SBP message.
--
maxObsPerMessage :: Int
maxObsPerMessage = (maxPayloadSize - headerSize) `div` packedObsSize
  where
    maxPayloadSize = 255
    headerSize     = 11
    packedObsSize  = 17

-- | The official GPS value of Pi. This is the value used by the CS to curve
-- fit ephemeris parameters and should be used in all ephemeris calculations.
gpsPi :: Double
gpsPi = 3.1415926535898

data SatelliteFrequency = SatelliteFrequency
  { _satelliteFrequencyL1 :: Double
  , _satelliteFrequencyL2 :: Double
  } deriving (Show, Eq)

gpsFrequency :: SatelliteFrequency
gpsFrequency = SatelliteFrequency l1frequency l2frequency

glonassChannelToFrequency :: Word8 -> Maybe SatelliteFrequency
glonassChannelToFrequency chan = M.lookup (fromIntegral chan - 7) mp
  where
    mp :: HashMap Int8 SatelliteFrequency
    mp = M.fromList
      [ (6, SatelliteFrequency 1.605375e9 1.248625e9)
      , (5, SatelliteFrequency 1.6048125e9 1.2481875e9)
      , (4, SatelliteFrequency 1.60425e9 1.24775e9)
      , (3, SatelliteFrequency 1.6036875e9 1.2473125e9)
      , (2, SatelliteFrequency 1.603125e9 1.246875e9)
      , (1, SatelliteFrequency 1.6025625e9 1.2464375e9)
      , (0, SatelliteFrequency 1.6020e9 1.2460e9)
      , (-1, SatelliteFrequency 1.6014375e9 1.2455625e9)
      , (-2, SatelliteFrequency 1.6008750e9 1.2451250e9)
      , (-3, SatelliteFrequency 1.6003125e9 1.2446875e9)
      , (-4, SatelliteFrequency 1.5997500e9 1.2442500e9)
      , (-5, SatelliteFrequency 1.5991875e9 1.2438125e9)
      , (-6, SatelliteFrequency 1.5986250e9 1.2433750e9)
      , (-7, SatelliteFrequency 1.5980625e9 1.2429375e9)
      ]


--------------------------------------------------------------------------------
-- General utilities


modifyMap :: (Eq k, Hashable k) => IORef (HashMap k v) -> v -> k -> (v -> v) -> IO v
modifyMap r d k a = do
  m <- readIORef r
  let v = a $ M.lookupDefault d k m
      n = M.insert k v m
  writeIORef r n
  n `seq` return v

maybe' :: Maybe a -> b -> (a -> b) -> b
maybe' m b a = maybe b a m

applyScaleFactor :: (Floating a, Integral b) => a -> a -> b -> a
applyScaleFactor n p x = (n ** p)  * fromIntegral x

--------------------------------------------------------------------------------
-- Lenses for abstracting over different observation types.
--
-- The measurement-related classes are all 'Getter'-only, because a
-- 'Lens'' couldn't expose the measurement as anything other than the
-- original integral type if we want the instance to handle
-- differences in representation between GPS and GLONASS.  For
-- example, the units of pseudorange ambiguity in GLONASS are twice
-- the size of those in GPS; to scale this properly, we must do it
-- after converting to a 'Double', which means we can never convert
-- back!

class GetL1CodeIndicator a where
  l1Code :: Getter a Bool

class GetL2CodeIndicator a where
  l2Code :: Getter a Word8

class GetPseudorange a where
  -- | Gets pseudorange in meters
  pseudorange :: Getter a Double

class GetPseudorangeAmbiguity a where
  -- | Gets pseudorange ambiguity in meters
  pseudorangeAmbiguity :: Getter a Double

class GetPseudorangeDifference a where
  -- | Gets pseudorange difference in meters
  pseudorangeDifference :: Getter a Double

class GetCarrierMinusCode a where
  -- | Gets carrier phase minus code in meters
  carrierMinusCode :: Getter a Double

class GetFrequency a where
  frequency :: a -> Maybe SatelliteFrequency

class HasLockTime a where
  lockTime :: Lens' a Word8

class HasCNR a where
  cnr :: Lens' a Word8

instance GetL1CodeIndicator GpsL1Observation where
  l1Code = gpsL1Observation_code

instance GetPseudorange GpsL1Observation where
  pseudorange =
    gpsL1Observation_pseudorange . to fromIntegral . multiplying twoCM

instance GetCarrierMinusCode GpsL1Observation where
  carrierMinusCode =
    gpsL1Observation_carrierMinusCode
    . to fromIntegral
    . multiplying phaseRangeRes

instance GetFrequency GpsL1Observation where
  frequency = const (pure gpsFrequency)

instance HasLockTime GpsL1Observation where
  lockTime = gpsL1Observation_lockTime

instance GetPseudorangeAmbiguity GpsL1ExtObservation where
  pseudorangeAmbiguity =
    gpsL1ExtObservation_ambiguity
    . to fromIntegral
    . multiplying lightSpeedMMSEC

instance HasCNR GpsL1ExtObservation where
  cnr = gpsL1ExtObservation_cnr

instance GetL2CodeIndicator GpsL2Observation where
  l2Code = gpsL2Observation_code

instance GetPseudorangeDifference GpsL2Observation where
  pseudorangeDifference =
    gpsL2Observation_pseudorangeDifference
    . to fromIntegral
    . multiplying twoCM

instance GetCarrierMinusCode GpsL2Observation where
  carrierMinusCode =
    gpsL2Observation_carrierMinusCode
    . to fromIntegral
    . multiplying phaseRangeRes

instance HasLockTime GpsL2Observation where
  lockTime = gpsL2Observation_lockTime

instance HasCNR GpsL2ExtObservation where
  cnr = gpsL2ExtObservation_cnr

instance GetL1CodeIndicator GlonassL1Observation where
  l1Code = glonassL1Observation_code

instance GetPseudorange GlonassL1Observation where
  pseudorange =
    glonassL1Observation_pseudorange . to fromIntegral . multiplying twoCM

instance GetCarrierMinusCode GlonassL1Observation where
  carrierMinusCode =
    glonassL1Observation_carrierMinusCode
    . to fromIntegral
    . multiplying phaseRangeRes

instance GetFrequency GlonassL1Observation where
  frequency gobs =
    gobs ^. glonassL1Observation_frequency . to glonassChannelToFrequency

instance HasLockTime GlonassL1Observation where
  lockTime = glonassL1Observation_lockTime

instance GetPseudorangeAmbiguity GlonassL1ExtObservation where
  pseudorangeAmbiguity =
    glonassL1ExtObservation_ambiguity
    . to fromIntegral
    . multiplying (lightSpeedMMSEC * 2)

instance HasCNR GlonassL1ExtObservation where
  cnr = glonassL1ExtObservation_cnr

instance GetL2CodeIndicator GlonassL2Observation where
  l2Code = glonassL2Observation_code

instance GetPseudorangeDifference GlonassL2Observation where
  pseudorangeDifference =
    glonassL2Observation_pseudorangeDifference
    . to fromIntegral
    . multiplying twoCM

instance GetCarrierMinusCode GlonassL2Observation where
  carrierMinusCode =
    glonassL2Observation_carrierMinusCode
    . to fromIntegral
    . multiplying phaseRangeRes

instance HasLockTime GlonassL2Observation where
  lockTime = glonassL2Observation_lockTime

instance HasCNR GlonassL2ExtObservation where
  cnr = glonassL2ExtObservation_cnr

--------------------------------------------------------------------------------
-- GNSS RTCM observation reconstruction utilities


fromEcefVal :: Int64 -> Double
fromEcefVal x = fromIntegral x / 10000

newGpsTime :: MonadStore e m => Word32 -> m GpsTimeNano
newGpsTime tow = do
  wn <- view storeWn >>= liftIO . readIORef
  return GpsTimeNano
    { _gpsTimeNano_tow         = tow
    , _gpsTimeNano_ns_residual = 0
    , _gpsTimeNano_wn          = wn
    }

-- | If incoming TOW is less than stored TOW, rollover WN.
--
updateGpsTime :: Word32 -> GpsTimeNano -> GpsTimeNano
updateGpsTime tow gpsTime =
  gpsTime & gpsTimeNano_tow .~ tow &
    if tow >= gpsTime ^. gpsTimeNano_tow then id else
      gpsTimeNano_wn %~ (+ 1)

-- | Produce GPS Time from Observation header, handling WN rollover.
--
toGpsTime :: MonadStore e m => GpsObservationHeader -> m GpsTimeNano
toGpsTime hdr = do
  let tow      = hdr ^. gpsObservationHeader_tow
      station  = hdr ^. gpsObservationHeader_station
  gpsTime      <- newGpsTime tow
  gpsTimeMap   <- view storeGpsTimeMap
  liftIO $ modifyMap gpsTimeMap gpsTime station $ updateGpsTime tow

{- GLONASS time shit -}

-- | An entry in the table of leap seconds
data UTCLeapSecond = UTCLeapSecond
  { _leapSecondWN     :: Int  -- ^ GPS WN of the leap second
  , _leapSecondTOW    :: Int  -- ^ GPS TOW of the leap second
  , leapSecondOffset  :: Int  -- ^ How many seconds GPS time is offset
                              -- from UTC time after the leap second
  } deriving ( Eq, Show, Read )

-- | Table of leap seconds through 2017.
utcLeapSecondTable :: [UTCLeapSecond]
utcLeapSecondTable =
  [ UTCLeapSecond 1930     17 18  -- 01-01-2017
  , UTCLeapSecond 1851 259216 17  -- 01-07-2015
  , UTCLeapSecond 1695     15 16  -- 01-07-2012
  , UTCLeapSecond 1512 345614 15  -- 01-01-2009
  , UTCLeapSecond 1356     13 14  -- 01-01-2006
  , UTCLeapSecond 990  432012 13  -- 01-01-1999
  , UTCLeapSecond 912  172811 12  -- 01-07-1997
  , UTCLeapSecond 834   86410 11  -- 01-01-1996
  , UTCLeapSecond 755  432009 10  -- 01-07-1994
  , UTCLeapSecond 703  345608  9  -- 01-07-1993
  , UTCLeapSecond 651  259207  8  -- 01-07-1992
  , UTCLeapSecond 573  172806  7  -- 01-01-1991
  , UTCLeapSecond 521   86405  6  -- 01-01-1990
  , UTCLeapSecond 416  432004  5  -- 01-01-1988
  , UTCLeapSecond 286   86403  4  -- 01-07-1985
  , UTCLeapSecond 181  432002  3  -- 01-07-1983
  , UTCLeapSecond 129  345601  2  -- 01-07-1982
  , UTCLeapSecond  77  259200  1  -- 01-07-1981
  ]

-- | Given a GPS time, compute the current offset from UTC time to GPS
-- time.
getUTCToGPSOffset :: Num c => GpsTime -> c
getUTCToGPSOffset t =
  fromIntegral . maybe 0 leapSecondOffset $
  find (timeAfterEntry t) utcLeapSecondTable
  where
    -- TODO(MP): The libswiftnav algorithm has a workaround for the
    -- case where a leap second is currently taking place, but this
    -- can never happen with ephemerides because they are always
    -- referenced to the edge of a 15-minute block.
    timeAfterEntry (GpsTime tow' wn') (UTCLeapSecond lwn ltow _)
      | lwn < wn = True
      | lwn > wn = False
      | ltow + 1 <= tow = True
      | otherwise = False
      where
        wn = fromIntegral wn'
        tow = fromIntegral tow'

-- | Convert a 'UTCTime' to a 'GpsTime' by correcting for leap
-- seconds.  There is no point in a 'GpsTimeNano', because we only
-- ever get GLONASS times to the millisecond.
utcToGpsTime :: UTCTime -> GpsTime
utcToGpsTime utctime = finalTime
  where
    -- Compute number of days since GPS epoch
    gpsDays = diffDays (utctDay utctime) (fromGregorian 1980 1 6)
    -- Compute week number
    wn  = fromIntegral $ gpsDays `div` 7
    dayTime = utctDayTime utctime
    -- Compute the time of week in MS.
    tow = diffTimeToMS dayTime +
          -- TOW due to days that have already passed.
          86400 * 1000 * fromIntegral (gpsDays `mod` 7)
    -- Remove the leap second if it's occurring right now
    towLeap = if (todSec . timeToTimeOfDay $ dayTime) > 60
              then 1000 else 0
    -- Compute time without leap seconds
    initialTime = normalizeGpsTime $ GpsTime (round tow - towLeap) wn
    -- Compute leap second offset
    initialOffset = getUTCToGPSOffset initialTime
    -- Compute whether we're missing another leap second between the
    -- start of this week and now
    weekStart = initialTime & gpsTime_tow .~ 0
    weekOffset = getUTCToGPSOffset weekStart
    weekCorrection = if weekOffset < initialOffset then 1000 else 0
    -- Add all the correction factors back in
    correctTime x = x + weekCorrection + towLeap + initialOffset * 1000
    finalTime = normalizeGpsTime $ initialTime & gpsTime_tow %~ correctTime

-- | Compute the number of milliseconds corresponding to the given
-- 'DiffTime'
diffTimeToMS :: DiffTime -> Double
diffTimeToMS dt =
  let TimeOfDay h m s = timeToTimeOfDay dt
      totalSec = toRational h * 3600 + toRational m * 60 + toRational s
  in fromRational (totalSec * 1000)

-- | This type represents a GLONASS system time.
newtype GlonassTime = GlonassTime { glonassTimeRepr :: UTCTime }
  deriving ( Show )

-- | Compute a GLONASS time from the GLONASS day and the number of
-- milliseconds into it.
mkGlonassTime :: Day -> Integer -> GlonassTime
mkGlonassTime d t =
  GlonassTime (UTCTime d (picosecondsToDiffTime $ 1000000000 * t))

-- | Convert a GLONASS ephemeris message to a 'GlonassTime' if
-- possible, or 'Nothing' if the ephemeris message doesn't contain the
-- appropriate information to determine the day.
glonassTimeFromGlonassEphemeris :: GlonassEphemeris -> Maybe GlonassTime
glonassTimeFromGlonassEphemeris ephem = do
  guard $ isM ephem
  -- Compute the current GLONASS year
  gloYear <- n4ntToYear
    (ephem ^. glonassEphemeris_mn4)
    (ephem ^. glonassEphemeris_mnt)
  -- Compute the current GLONASS day of the year
  gloDayOfYear <- ntToDay (ephem ^. glonassEphemeris_mnt)
  let
    yearStartDay = fromGregorian gloYear 0 0
    -- Subtract one from the day of year; we don't want to add all of
    -- today
    glonassDay = (fromIntegral gloDayOfYear - 1) `addDays` yearStartDay
  return $
    mkGlonassTime glonassDay (glonassMSFromEphemeris ephem)
  where
    isM x = x ^. glonassEphemeris_mM == 1
    -- Convert the GLONASS-M N4 field to a year
    n4ntToYear n4 nt
      | n4 < 1    = Nothing
      | n4 > 31   = Nothing
      | otherwise = do
          cycleYear <- ntToYear nt
          return $ 1996 + 4 * (fromIntegral n4 - 1) + (cycleYear - 1)
    -- Convert the GLONASS-M Nt field to a year counter
    ntToYear nt
      | nt < 1              = Nothing
      | nt <= 366           = Just 1
      | nt <= 366 + 365     = Just 2
      | nt <= 366 + 365 * 2 = Just 3
      | nt <= 366 + 365 * 3 = Just 4
      | otherwise           = Nothing
    -- Convert the GLONASS-M Nt field to a day-of-the-year offset
    ntToDay nt
      | nt < 1              = Nothing
      | nt <= 366           = Just nt
      | nt <= 366 + 365     = Just $ nt - 366
      | nt <= 366 + 365 * 2 = Just $ nt - (366 + 365)
      | nt <= 366 + 365 * 3 = Just $ nt - (366 + 365 * 2)
      | otherwise           = Nothing

getStoreTime :: MonadStore e m => m UTCTime
getStoreTime = view storeUTCTime >>= liftIO . readIORef

-- | Convert a GLONASS milliseconds-in-day count to a 'GlonassTime',
-- using the stored UTC time to figure out what day it is.
glonassTimeFromStore :: Word32 -> UTCTime -> GlonassTime
glonassTimeFromStore glonassMS storeTime@(UTCTime storeDay _) =
  -- Form an absolute GLONASS epoch
  mkGlonassTime (addDays dayCorrection storeDay) (fromIntegral glonassMS)
  where
    -- Compute the store time in GLONASS time frame
    storeTimeGlonass = utcTimeToGlonass storeTime
    -- Extract the number of milliseconds of the current GLONASS day
    -- of the store time
    storeMS = diffTimeToMS . utctDayTime . glonassTimeRepr $ storeTimeGlonass
    -- Figure out where in the GLONASS day we are
    dayCorrection = if glonassMS < round storeMS
                    -- Stream time is behind the program start time,
                    -- so we must have wrapped a day.
                    then 1
                    -- Stream time is after the program start time, so
                    -- we have the correct day.
                    else 0

-- | Convert a 'UTCTime' to a 'GlonassTime' by adding the offset
utcTimeToGlonass :: UTCTime -> GlonassTime
utcTimeToGlonass = GlonassTime . addUTCTime (glonassUTCDelta)
  where
    glonassUTCDelta = 3 * 3600

-- | Convert a 'GlonassTime' to a 'UTCTime' by removing the offset
glonassToUTCTime :: GlonassTime -> UTCTime
glonassToUTCTime = addUTCTime (-glonassUTCDelta) . glonassTimeRepr
  where
    glonassUTCDelta = 3 * 3600

glonassObsToGpsTimeNano :: MonadStore e m => GlonassObservationHeader -> m GpsTimeNano
glonassObsToGpsTimeNano hdr = do
  gloEpoch <- glonassTimeFromStore (hdr ^. glonassObservationHeader_epoch) <$> getStoreTime
  let GpsTime gpsTow gpsWn = utcToGpsTime $ glonassToUTCTime gloEpoch
  return $ GpsTimeNano gpsTow 0 gpsWn

-- | Produce GPS time from RTCM time (week-number % 1024, tow in seconds, scaled 2^4).
-- Stateful but non-side-effecting. The week number is mod-1024 - add in the base
-- wn from the stored wn masked to 10 bits.
rtcmTimeToGpsTime :: MonadStore e m => Word16 -> Word16 -> m GpsTime
rtcmTimeToGpsTime wn tow = do
  stateWn <- view storeWn >>= liftIO . readIORef
  return $ GpsTime
    { _gpsTime_tow = 16 * fromIntegral tow
    , _gpsTime_wn  = stateWn `shiftR` 10 `shiftL` 10 + wn
    }

-- | Compute the milliseconds into the day representing the time of
-- ephemeris in a given GLONASS ephemeris message
glonassMSFromEphemeris :: Integral t => GlonassEphemeris -> t
glonassMSFromEphemeris ephem =
  fromIntegral (ephem ^. glonassEphemeris_tb) * 15 * msPerMinute
  where
    msPerMinute = 60 * 1000

-- | Normalizes (@tow@ < 1 week, @tow@ > 0) a GPS time
normalizeGpsTime :: GpsTime -> GpsTime
normalizeGpsTime t@(GpsTime tow wn)
  -- TODO(MP): what happens if there is a leap second during the week?
  -- Do we incorrectly truncate the TOW?
  | tow >= 7 * msPerDay =
      normalizeGpsTime $ GpsTime (tow - msPerDay) (wn + 1)
  | tow < 0             =
      normalizeGpsTime $ GpsTime (tow + msPerDay) (wn - 1)
  | otherwise           = t
  where
    msPerDay = 86400 * 1000

-- | Compute a 'GpsTime' from the given GLONASS ephemeris message.
glonassEphemerisToGpsTime :: MonadStore e m => Msg1020 -> m GpsTime
glonassEphemerisToGpsTime (Msg1020 _ ephem) = do
  -- What do we do here?  Attempt to compute the GLONASS time based
  -- solely on the RTCM stream.  If the stream is only non-GLONASS-M
  -- messages, which happens often (IGS, AUSCORS, Skylark) in spite of
  -- there being no pre-GLONASS-M satellites in operation since 2005,
  -- we fall back on the store time to give the day.
  glonassTime <- maybe (storetime ephem) pure $ ephemtime ephem
  -- Now convert from GLONASS to UTC to GPS
  return . utcToGpsTime . glonassToUTCTime $ glonassTime
  where
    storetime eph =
      glonassTimeFromStore (glonassMSFromEphemeris eph) <$> getStoreTime
    ephemtime = glonassTimeFromGlonassEphemeris

-- | Lookup Table 4.4 from GLONASS ICD: convert F_T word to accuracy in meters
glonassFtMeters :: (Num b, Eq b, Fractional a) => b -> Maybe a
glonassFtMeters 0 = Just 1
glonassFtMeters 1 = Just 2
glonassFtMeters 2 = Just 2.5
glonassFtMeters 3 = Just 4
glonassFtMeters 4 = Just 5
glonassFtMeters 5 = Just 7
glonassFtMeters 6 = Just 10
glonassFtMeters 7 = Just 12
glonassFtMeters 8 = Just 14
glonassFtMeters 9 = Just 16
glonassFtMeters 10 = Just 32
glonassFtMeters 11 = Just 64
glonassFtMeters 12 = Just 128
glonassFtMeters 13 = Just 256
glonassFtMeters 14 = Just 512
glonassFtMeters _ = Nothing

glonassEphemerisValid :: GlonassEphemeris -> Bool
glonassEphemerisValid e
  -- Almanac health info is available
  =  e ^. glonassEphemeris_healthAvailability
  -- Almanac health is good
  && e ^. glonassEphemeris_almanacHealth
  -- Ephemeris health is good
  && e ^. glonassEphemeris_bn_msb

glonassFitInterval :: GlonassEphemeris -> Maybe Word32
glonassFitInterval e =
  (60 *) <$> case e ^. glonassEphemeris_p1 of
               0 -> Just 0
               1 -> Just 30
               2 -> Just 45
               3 -> Just 60
               _ -> Nothing

-- | MJD GPS Epoch - First day in GPS week 0. See DF051 of the RTCM3 spec
--
mjdEpoch :: Word16
mjdEpoch = 44244

-- | Convert from MJD to GPS week number
--
-- See DF051 of the RTCM3 spec
toWn :: Word16 -> Word16
toWn mjd = (mjd - mjdEpoch) `div` 7

-- | Determine whether an L1 RTCM observation is invalid.
--
-- See DF011 and DF012 of the RTCM3 spec
invalid_L1 :: GpsL1Observation -> Bool
invalid_L1 l1 =
  l1 ^. gpsL1Observation_pseudorange == 524288 ||
  l1 ^. gpsL1Observation_carrierMinusCode == -524288

-- | Determine whether an L1 + L2 RTCM observation is invalid.
--
-- See DF011, DF012, and DF018 of the RTCM3 spec
invalid_L2 :: GpsL2Observation -> Bool
invalid_L2 l2 =
  l2 ^. gpsL2Observation_pseudorangeDifference == -8192 ||
  l2 ^. gpsL2Observation_carrierMinusCode == -524288

-- | See DF042 of the RTCM3 spec
glonassCarrierValidL1 :: GlonassL1Observation -> Bool
glonassCarrierValidL1 obs =
  obs ^. carrierMinusCode /= 0x80000

-- | See DF047 of the RTCM3 spec
glonassPseudorangeValidL2 :: GlonassL2Observation -> Bool
glonassPseudorangeValidL2 obs =
  obs ^. pseudorangeDifference /= 0x2000

-- | See DF048 of the RTCM3 spec
glonassCarrierValidL2 :: GlonassL2Observation -> Bool
glonassCarrierValidL2 obs =
  obs ^. carrierMinusCode /= 0x80000

-- | Construct metric pseudorange (meters!) from L1 RTCM observation.
--
-- See DF011, pg. 3-16 of the RTCM3 spec
metricPseudorange :: GpsL1Observation -> GpsL1ExtObservation -> Double
metricPseudorange l1 l1e =
  twoCM * fromIntegral (l1 ^. gpsL1Observation_pseudorange) +
  lightSpeedMMSEC * fromIntegral (l1e ^. gpsL1ExtObservation_ambiguity)

metricPseudorange' :: (GetPseudorange a, GetPseudorangeAmbiguity b)
                   => a -> b -> Double
metricPseudorange' obs obsExt =
  obs ^. pseudorange + obsExt ^. pseudorangeAmbiguity

-- | Construct L1 SBP pseudorange for L1 RTCM observation
--
-- See DF011, pg. 3-16 of the RTCM3 spec
toP_L1 :: GpsL1Observation -> GpsL1ExtObservation -> Word32
toP_L1 l1 l1e = round $ sbpCMinM * metricPseudorange l1 l1e

toP_L1' :: (GetPseudorange a, GetPseudorangeAmbiguity b) => a -> b -> Word32
toP_L1' l1 l1e = round $ sbpCMinM * metricPseudorange' l1 l1e

-- | Construct L2 SBP pseudorange from L1/L2 RTCM observations
--
-- See DF017, pg. 3-18 of the RTCM3 spec
toP_L2 :: GpsL1Observation -> GpsL1ExtObservation -> GpsL2Observation -> Word32
toP_L2 l1 l1e l2 = round $ p * sbpCMinM where
  p = metricPseudorange l1 l1e +
      twoCM * fromIntegral (l2 ^. gpsL2Observation_pseudorangeDifference)

toP_L2' :: ( GetPseudorange a
           , GetPseudorangeAmbiguity b
           , GetPseudorangeDifference c)
        => a -> b -> c -> Word32
toP_L2' l1 l1e l2 = round $ p * sbpCMinM where
  p = metricPseudorange' l1 l1e + l2 ^. pseudorangeDifference

-- | Construct SBP L1 GPS carrier phase from L1 RTCM observation
--
-- See DF012, pg. 3-16 of the RTCM3 spec
toL_L1 :: GpsL1Observation -> GpsL1ExtObservation -> CarrierPhase
toL_L1 l1 l1e = CarrierPhase
  { _carrierPhase_i = fromIntegral li'
  , _carrierPhase_f = fromIntegral lf'
  } where
    p = metricPseudorange l1 l1e
    -- Convert to SBP carrier phase representation per
    -- https://github.com/swift-nav/libsbp/blob/master/spec/yaml/swiftnav/sbp/observation.yaml#L39
    lm :: Double
    lm = p + phaseRangeRes * fromIntegral (l1 ^. gpsL1Observation_carrierMinusCode)
    l = lm / (lightSpeedMS / l1frequency)
    li :: Int32
    li = floor l
    lf :: Word16
    lf = round ((l - fromIntegral li) * q32Width)
    li' :: Int32
    li' = if lf == 256 then li + 1 else li
    lf' :: Word8
    lf' = if lf == 256 then 0 else fromIntegral lf

-- | Construct SBP L1 GPS carrier phase from L1 RTCM observation
--
-- See DF012, pg. 3-16 of the RTCM3 spec
toL_L1' :: ( GetPseudorange a
           , GetCarrierMinusCode a
           , GetFrequency a
           , GetPseudorangeAmbiguity b) => a -> b -> Maybe CarrierPhase
toL_L1' l1 l1e = do
  freq <- _satelliteFrequencyL1 <$> frequency l1
  let
    p = metricPseudorange' l1 l1e
    -- Convert to SBP carrier phase representation per
    -- https://github.com/swift-nav/libsbp/blob/master/spec/yaml/swiftnav/sbp/observation.yaml#L39
    lm :: Double
    lm = p + l1 ^. carrierMinusCode
    l = lm / (lightSpeedMS / freq)
    li :: Int32
    li = floor l
    lf :: Word16
    lf = round ((l - fromIntegral li) * q32Width)
    li' :: Int32
    li' = if lf == 256 then li + 1 else li
    lf' :: Word8
    lf' = if lf == 256 then 0 else fromIntegral lf
  return $ CarrierPhase
    { _carrierPhase_i = fromIntegral li'
    , _carrierPhase_f = fromIntegral lf'
    }

-- | Construct SBP L2 GPS carrier phase from L2 RTCM observation
--
-- See DF018, pg. 3-18 of the RTCM3 spec
toL_L2 :: GpsL1Observation
       -> GpsL1ExtObservation
       -> GpsL2Observation
       -> GpsL2ExtObservation
       -> CarrierPhase
toL_L2 l1 l1e l2 _l2e = CarrierPhase
  { _carrierPhase_i = fromIntegral li
  , _carrierPhase_f = fromIntegral lf
  } where
    p = metricPseudorange l1 l1e
    -- Convert to SBP carrier phase representation per
    -- https://github.com/swift-nav/libsbp/blob/master/spec/yaml/swiftnav/sbp/observation.yaml#L39
    lm :: Double
    lm = p + phaseRangeRes * fromIntegral (l2 ^. gpsL2Observation_carrierMinusCode)
    l = lm / (lightSpeedMS / l2frequency)
    li :: Int32
    li = floor l
    lf :: Word8
    lf = round ((l - fromIntegral li) * q32Width)

toL_L2' :: ( GetPseudorange a
           , GetFrequency a
           , GetCarrierMinusCode a
           , GetPseudorangeAmbiguity b
           , GetCarrierMinusCode c)
        => a -> b -> c -> d -> Maybe CarrierPhase
toL_L2' l1 l1e l2 _l2e = do
  freq <- _satelliteFrequencyL2 <$> frequency l1
  let
    p = metricPseudorange' l1 l1e
    -- Convert to SBP carrier phase representation per
    -- https://github.com/swift-nav/libsbp/blob/master/spec/yaml/swiftnav/sbp/observation.yaml#L39
    lm :: Double
    lm = p + l2 ^. carrierMinusCode
    l = lm / (lightSpeedMS / freq)
    li :: Int32
    li = floor l
    lf :: Word8
    lf = round ((l - fromIntegral li) * q32Width)
  return CarrierPhase
    { _carrierPhase_i = fromIntegral li
    , _carrierPhase_f = fromIntegral lf
    }

toCn0_L1 :: GpsL1ExtObservation -> Word8
toCn0_L1 = (^. gpsL1ExtObservation_cnr)

toCn0_L2 :: GpsL2ExtObservation -> Word8
toCn0_L2 = (^. gpsL2ExtObservation_cnr)

toCn0 :: HasCNR a => a -> Word8
toCn0 = (^. cnr)

-- | Convert between DF013 and DF019 lock time to DF402 lock time
--
toLock :: Word8 -> Word8
toLock t
  | t' < 32     = 0
  | t' < 64     = 1
  | t' < 128    = 2
  | t' < 256    = 3
  | t' < 512    = 4
  | t' < 1024   = 5
  | t' < 2048   = 6
  | t' < 4096   = 7
  | t' < 8192   = 8
  | t' < 16384  = 9
  | t' < 32768  = 10
  | t' < 65536  = 11
  | t' < 131072 = 12
  | t' < 262144 = 13
  | t' < 524288 = 14
  | otherwise   = 15
  where
    ti = fromIntegral t
    t', t'' :: Word32
    t' = 1000 * t''
    t''
      | t <= 23   = ti
      | t <= 47   = ti * 2 - 24
      | t <= 71   = ti * 4 - 120
      | t <= 95   = ti * 8 - 408
      | t <= 119  = ti * 16 - 1176
      | t <= 126  = ti * 32 - 3096
      | otherwise = 937

-- | Construct sequenced SBP observation header
--
fromGpsObservationHeader :: MonadStore e m
                         => Word8                -- ^ Total messages
                         -> Word8                -- ^ Message in sequence
                         -> GpsObservationHeader -- ^ RTCM observation header
                         -> m ObservationHeader
fromGpsObservationHeader totalMsgs n hdr = do
  t <- toGpsTime hdr
  return ObservationHeader
    { _observationHeader_t     = t
    -- First nibble is the size of the sequence (n), second nibble is the
    -- zero-indexed counter (ith packet of n). See observation header packing
    -- https://github.com/swift-nav/libsbp/blob/master/spec/yaml/swiftnav/sbp/observation.yaml#L63
    , _observationHeader_n_obs = totalMsgs `shiftL` 4 .|. n
    }

-- | Construct sequenced SBP observation header
--
fromGlonassObservationHeader :: MonadStore e m
                             => Word8                    -- ^ Total messages
                             -> Word8                    -- ^ Message in sequence
                             -> GlonassObservationHeader -- ^ RTCM observation header
                             -> m ObservationHeader
fromGlonassObservationHeader totalMsgs n hdr = do
  t <- glonassObsToGpsTimeNano hdr
  return ObservationHeader
    { _observationHeader_t     = t
    -- First nibble is the size of the sequence (n), second nibble is the
    -- zero-indexed counter (ith packet of n). See observation header packing
    -- https://github.com/swift-nav/libsbp/blob/master/spec/yaml/swiftnav/sbp/observation.yaml#L63
    , _observationHeader_n_obs = totalMsgs `shiftL` 4 .|. n
    }

-- | Construct an L1 GnssSignal
--
toL1GnssSignal :: Word8 -> GpsL1Observation -> GnssSignal16
toL1GnssSignal sat l1 =
  GnssSignal16
    { _gnssSignal16_sat  = sat
    , _gnssSignal16_code = if l1 ^. gpsL1Observation_code then l1PSidCode else l1CSidCode
    }

-- | Construct an L1 SBP PackedObsContent an RTCM satellite vehicle observation
--
fromL1SatelliteObservation :: MonadStore e m
                           => Word8               -- ^ Satellite PRN
                           -> GpsL1Observation
                           -> GpsL1ExtObservation
                           -> m PackedObsContent
fromL1SatelliteObservation sat l1 l1e = do
  -- Checks GPS L1 code indicator for RTCM message 1002.
  -- See DF010, pg. 3-17 of the RTCM3 spec.
  let sid = toL1GnssSignal sat l1
  return PackedObsContent
    { _packedObsContent_P     = toP_L1 l1 l1e
    , _packedObsContent_L     = toL_L1 l1 l1e
    , _packedObsContent_D     = Doppler 0 0
    , _packedObsContent_cn0   = toCn0_L1 l1e
    , _packedObsContent_lock  = toLock $ l1 ^. gpsL1Observation_lockTime
    , _packedObsContent_sid   = sid
    , _packedObsContent_flags = 0x7 -- Doppler Invalid
                                    -- 1/2 cycle phase ambiguity resolved
                                    -- carrier valid
                                    -- pseudorange valid
    }

fromL1GlonassObservation :: MonadStore e m
                         => Word8
                         -> GlonassL1Observation
                         -> GlonassL1ExtObservation
                         -> m (Maybe PackedObsContent)
fromL1GlonassObservation sat l1 l1e = return $ do
  l1l <- toL_L1' l1 l1e
  return PackedObsContent
    { _packedObsContent_P = toP_L1' l1 l1e
    , _packedObsContent_L = l1l
    , _packedObsContent_D = Doppler 0 0
    , _packedObsContent_cn0 = toCn0 l1e
    , _packedObsContent_lock = toLock $ l1 ^. lockTime
    , _packedObsContent_sid = sid
      -- Doppler is always invalid; pseudorange is always valid.
    , _packedObsContent_flags = computeFlags
    }
  where
    -- carrier valid (bit 1) and half-cycle ambiguity resolved (bit 2)
    computeFlags = 0x1 .|. if glonassCarrierValidL1 l1 then 0x6 else 0
    sid = glonassToL1GnssSignal sat l1

glonassToL1GnssSignal :: Word8 -> GlonassL1Observation -> GnssSignal16
glonassToL1GnssSignal sat l1 =
  GnssSignal16
    { _gnssSignal16_sat = sat
    , _gnssSignal16_code =
      -- For now, there is no support for L1P in libswiftnav, so we
      -- signal an invalid code.
      if not $ l1 ^. glonassL1Observation_code
      then glonassL1CASidCode
      else 0xff
    }

-- | Construct an L2 GnssSignal
--
toL2GnssSignal :: Word8 -> GpsL2Observation -> Maybe GnssSignal16
toL2GnssSignal sat l2 = do
  code <- M.lookup (l2 ^. gpsL2Observation_code) l2codeToSBPSignalCode
  return GnssSignal16
    { _gnssSignal16_sat  = fromIntegral sat
    , _gnssSignal16_code = code
    }

-- | Construct an L2 SBP PackedObsContent an RTCM satellite vehicle observation
--
fromL2SatelliteObservation :: MonadStore e m
                           => Word8                  -- ^ Satellite PRN
                           -> GpsL1Observation
                           -> GpsL1ExtObservation
                           -> GpsL2Observation
                           -> GpsL2ExtObservation
                           -> m (Maybe PackedObsContent)
fromL2SatelliteObservation sat l1 l1e l2 l2e =
  -- Checks GPS L2 code indicator.
  -- See DF016, pg. 3-17 of the RTCM3 spec.
  maybe' (toL2GnssSignal sat l2) (return Nothing) $ \sid -> do
    return $ Just PackedObsContent
      { _packedObsContent_P     = toP_L2 l1 l1e l2
      , _packedObsContent_L     = toL_L2 l1 l1e l2 l2e
      , _packedObsContent_D     = Doppler 0 0
      , _packedObsContent_cn0   = toCn0_L2 l2e
      , _packedObsContent_lock  = toLock $ l2 ^. gpsL2Observation_lockTime
      , _packedObsContent_sid   = sid
      , _packedObsContent_flags = 0x7 -- Doppler Invalid
                                      -- 1/2 cycle phase ambiguity resolved
                                      -- carrier valid
                                      -- pseudorange valid
    }

glonassToL2GnssSignal :: Word8 -> GlonassL2Observation -> GnssSignal16
glonassToL2GnssSignal sat l2 =
  let code = l2 ^. l2Code
      markCode c
        | c == 0 = glonassL2CASidCode
          -- There is no support for L2P in libswiftnav, but according
          -- to the experts, it doesn't actually matter whether we
          -- track the P code or the C/A code -- we'll get the same
          -- answer.  So for now, we'll mark this as L2C/A anyway.
        | c == 1 = glonassL2CASidCode
        | otherwise = 0xff
  in GnssSignal16
     { _gnssSignal16_sat = sat
     , _gnssSignal16_code = markCode code
     }

fromL2GlonassObservation :: MonadStore e m
                         => Word8
                         -> GlonassL1Observation
                         -> GlonassL1ExtObservation
                         -> GlonassL2Observation
                         -> GlonassL2ExtObservation
                         -> m (Maybe PackedObsContent)
fromL2GlonassObservation sat l1 l1e l2 l2e = return $ do
  -- Checks GPS L2 code indicator.
  -- See DF016, pg. 3-17 of the RTCM3 spec.
  l2l <- toL_L2' l1 l1e l2 l2e
  return $ PackedObsContent
    { _packedObsContent_P     = toP_L2' l1 l1e l2
    , _packedObsContent_L     = l2l
    , _packedObsContent_D     = Doppler 0 0
    , _packedObsContent_cn0   = toCn0 l2e
    , _packedObsContent_lock  = toLock $ l2 ^. lockTime
    , _packedObsContent_sid   = sid
    , _packedObsContent_flags = computeFlags
    }
  where
    sid = glonassToL2GnssSignal sat l2
    -- Doppler is always invalid
    computeFlags = carrierFlag .|. pseudorangeFlag
    -- carrier valid (bit 1) and half-cycle ambiguity resolved (bit 2)
    carrierFlag = if glonassCarrierValidL2 l2 then 0x6 else 0
    -- pseudorange valid (bit 0)
    pseudorangeFlag = if glonassPseudorangeValidL2 l2 then 0x1 else 0

-- | Validate IODE/IODC flags. The IODC and IODE flags (least significant bits) should be
-- equal. IODC is Word16 while IODE is Word8, so we shift IODC to drop the most significant bits.
-- We return a 1 if valid, 0 if invalid.
validateIodcIode :: Word16 -> Word8 -> Word8
validateIodcIode iodc iode =
  if iodc' == iode then 1 else 0
  where
    iodc' = fromIntegral $ iodc `shiftL` 8 `shiftR` 8

-- | Construct an EphemerisCommonContent from an RTCM 1019 message.
toGpsEphemerisCommonContent :: MonadStore e m => Msg1019 -> m EphemerisCommonContent
toGpsEphemerisCommonContent m = do
  toe <- rtcmTimeToGpsTime (m ^. msg1019_ephemeris ^. gpsEphemeris_wn) (m ^. msg1019_ephemeris ^. gpsEphemeris_toe)
  return EphemerisCommonContent
    { _ephemerisCommonContent_sid = GnssSignal16
      { _gnssSignal16_sat  = m ^. msg1019_header ^. gpsEphemerisHeader_sat
      , _gnssSignal16_code = 0 -- there is an L2P status flag in msg 1019, but I don't think that applies
      }
    , _ephemerisCommonContent_toe          = toe
    , _ephemerisCommonContent_ura          = gpsUriToUra (fromIntegral $ m ^. msg1019_ephemeris ^. gpsEphemeris_svHealth)
    , _ephemerisCommonContent_fit_interval = decodeFitInterval (m ^. msg1019_ephemeris ^. gpsEphemeris_fitInterval) (m ^. msg1019_ephemeris ^. gpsEphemeris_iodc)
    , _ephemerisCommonContent_valid        = validateIodcIode (m ^. msg1019_ephemeris ^. gpsEphemeris_iodc) (m ^. msg1019_ephemeris ^. gpsEphemeris_iode)
    , _ephemerisCommonContent_health_bits  = m ^. msg1019_ephemeris ^. gpsEphemeris_svHealth
    }

glonassToEphemerisCommonContent :: MonadStore e m
                                => Msg1020
                                -> m (Maybe EphemerisCommonContent)
glonassToEphemerisCommonContent m = do
  toe <- glonassEphemerisToGpsTime m
  return $ do
    ura <- glonassFtMeters $ eph ^. glonassEphemeris_mft
    fit_interval <- glonassFitInterval eph
    return EphemerisCommonContent
      { _ephemerisCommonContent_sid = GnssSignal16
        { _gnssSignal16_sat      = fromIntegral $ m ^. msg1020_header . glonassEphemerisHeader_sat - 1
        , _gnssSignal16_code     = 3
        }
        -- We have to emit a 'GpsTime' whose time is measured in
        -- seconds for this message.  We shouldn't need to care about
        -- rounding the milliseconds, as ephemeris times are always
        -- provided on the round second (after leap-second conversion)
      , _ephemerisCommonContent_toe = toe & gpsTime_tow %~ (`div` 1000)
      , _ephemerisCommonContent_ura = ura
      , _ephemerisCommonContent_fit_interval = fit_interval
      , _ephemerisCommonContent_valid = boolToWord8 (glonassEphemerisValid eph)
      , _ephemerisCommonContent_health_bits =
          boolToWord8 $ eph ^. glonassEphemeris_mln5 || eph ^. glonassEphemeris_mi3
      }
  where
    eph = m ^. msg1020_ephemeris
    boolToWord8 = bool 1 0


-- | Construct SBP GPS observation message (possibly chunked).
--
chunkToMsgObs :: MonadStore e m
              => GpsObservationHeader -- ^ RTCM observation header
              -> Word8                -- ^ Total messages
              -> Word8                -- ^ Message in sequence
              -> [PackedObsContent]
              -> m MsgObs
chunkToMsgObs hdr totalMsgs n packed = do
  header <- fromGpsObservationHeader totalMsgs n hdr
  return MsgObs
    { _msgObs_header = header
    , _msgObs_obs    = packed
    }

-- | Construct SBP GPS observation message (possibly chunked).
--
glonassChunkToMsgObs :: MonadStore e m
                     => GlonassObservationHeader -- ^ RTCM observation header
                     -> Word8                    -- ^ Total messages
                     -> Word8                    -- ^ Message in sequence
                     -> [PackedObsContent]
                     -> m MsgObs
glonassChunkToMsgObs hdr totalMsgs n packed = do
  header <- fromGlonassObservationHeader totalMsgs n hdr
  return MsgObs
    { _msgObs_header = header
    , _msgObs_obs    = packed
    }

-- | 16 bit SBP Sender Id is 12 bit RTCM Station Id with high nibble or'd in
--
toSender :: Word16 -> Word16
toSender = (.|. 0xf000)

-- | Decode SBP 'fitInterval' from RTCM/GPS 'fitIntervalFlag' and IODC.
-- Implementation adapted from libswiftnav/ephemeris.c.
decodeFitInterval :: Bool -> Word16 -> Word32
decodeFitInterval fitInt iodc
  | not fitInt                                                     = 4 * 60 * 60
  | iodc >= 240 && iodc <= 247                                     = 8 * 60 * 60
  | (iodc >= 248 && iodc <= 255) || iodc == 496                    = 14 * 60 * 60
  | (iodc >= 497 && iodc <= 503) || (iodc >= 1021 && iodc <= 1023) = 26 * 60 * 60
  | iodc >= 504 && iodc <= 510                                     = 50 * 60 * 60
  | iodc == 511 || (iodc >= 752 && iodc <= 756)                    = 74 * 60 * 60
  | iodc == 757                                                    = 98 * 60 * 60
  | otherwise                                                      = 6 * 60 * 60

-- | Convert between RTCM/GPS URA ("User Range Accuracy") index to a number in meters.
-- See section 2.5.3, "User Range Accuracy", in the GPS signal specification.
-- Indices 1, 3, and 5 are hard-coded according to spec, and 15 is hard-coded according
-- to SBP/Piksi convention.
gpsUriToUra :: Double -> Double
gpsUriToUra uri
  | uri < 0             = -1
  | uri == 1            = 2.8
  | uri == 3            = 5.7
  | uri == 5            = 11.3
  | uri == 15           = 6144
  | uri <= 6            = 2 ** (1 + (uri / 2))
  | uri > 6 && uri < 15 = 2 ** (uri - 2)
  | otherwise           = -1

--------------------------------------------------------------------------------
-- RTCM to SBP conversion utilities: RTCM Msgs. 1002 (L1 RTK), 1004 (L1+L2 RTK),
-- 1005 (antenna position), 1006 (antenna position).


-- | Construct an L1 SBP PackedObsContent from an RTCM Msg 1002.
--
fromObservation1002 :: MonadStore e m => Observation1002 -> m [PackedObsContent]
fromObservation1002 obs = do
  obs1 <- fromL1SatelliteObservation sat l1 l1e
  return $
    -- Only lower set of PRN numbers (1-32) are supported
    if sat > maxSats then mempty else
      if invalid_L1 l1 then mempty else
        [obs1]
  where
    sat = obs ^. observation1002_sat
    l1  = obs ^. observation1002_l1
    l1e = obs ^. observation1002_l1e


-- | Convert an RTCM L1 1002 observation into an SBP MsgObs.
--
-- This chunking takes places because the number of observations in a given 1002
-- may very well exceed the maximum SBP supported payload size of 255 bytes.
fromMsg1002 :: MonadStore e m => Msg1002 -> m [MsgObs]
fromMsg1002 m = do
  let hdr = m ^. msg1002_header
  obs <- concatMapM fromObservation1002 $ m ^. msg1002_observations
  let chunks    = zip [0..] $ chunksOf maxObsPerMessage obs
      totalMsgs = fromIntegral $ length chunks
  forM chunks $ uncurry $ chunkToMsgObs hdr totalMsgs

-- | Construct an L1/L2 SBP PackedObsContent from an RTCM Msg 1004.
--
fromObservation1004 :: MonadStore e m => Observation1004 -> m [PackedObsContent]
fromObservation1004 obs = do
  obs1 <- fromL1SatelliteObservation sat l1 l1e
  obs2 <- fromL2SatelliteObservation sat l1 l1e l2 l2e
  return $
    -- Only lower set of PRN numbers (1-32) are supported
    if sat > maxSats then mempty else
      if invalid_L1 l1 && invalid_L2 l2 then mempty else
        if invalid_L1 l1 then maybeToList obs2 else
          if invalid_L2 l2 then [obs1] else
            obs1 : maybeToList obs2
  where
    sat = obs ^. observation1004_sat
    l1  = obs ^. observation1004_l1
    l1e = obs ^. observation1004_l1e
    l2  = obs ^. observation1004_l2
    l2e = obs ^. observation1004_l2e

fromObservation1012 :: MonadStore e m => Observation1012 -> m [PackedObsContent]
fromObservation1012 obs = do
  obs1 <- fromL1GlonassObservation sat l1 l1e
  obs2 <- fromL2GlonassObservation sat l1 l1e l2 l2e
  return $ singleObs obs1 <> singleObs obs2
  where
    singleObs = maybe [] pure
    sat = obs ^. observation1012_sat
    l1  = obs ^. observation1012_l1
    l1e = obs ^. observation1012_l1e
    l2  = obs ^. observation1012_l2
    l2e = obs ^. observation1012_l2e

-- fromObservation1012 :: MonadStore e m => Observation1012 -> m [PackedObsContent]
-- fromObservation1012 obs = do


-- | Convert an RTCM L1+L2 1004 observation into multiple SBP MsgObs.
--
-- This chunking takes places because the number of observations in a given 1004
-- may very well exceed the maximum SBP supported payload size of 255 bytes.
fromMsg1004 :: MonadStore e m => Msg1004 -> m [MsgObs]
fromMsg1004 m = do
  let hdr = m ^. msg1004_header
  obs <- concatMapM fromObservation1004 $ m ^. msg1004_observations
  let chunks    = zip [0..] $ chunksOf maxObsPerMessage obs
      totalMsgs = fromIntegral $ length chunks
  forM chunks $ uncurry $ chunkToMsgObs hdr totalMsgs

-- | Convert an RTCM 1005 antenna reference position message into an SBP
-- MsgBasePosEcef.
fromMsg1005 :: MonadStore e m => Msg1005 -> m MsgBasePosEcef
fromMsg1005 m =
  return MsgBasePosEcef
    { _msgBasePosEcef_x = fromEcefVal $ m ^. msg1005_reference ^. antennaReference_ecef_x
    , _msgBasePosEcef_y = fromEcefVal $ m ^. msg1005_reference ^. antennaReference_ecef_y
    , _msgBasePosEcef_z = fromEcefVal $ m ^. msg1005_reference ^. antennaReference_ecef_z
    }

-- | Convert an RTCM 1006 antenna reference position message into an SBP
-- MsgBasePosEcef.
fromMsg1006 :: MonadStore e m => Msg1006 -> m MsgBasePosEcef
fromMsg1006 m =
  return MsgBasePosEcef
    { _msgBasePosEcef_x = fromEcefVal $ m ^. msg1006_reference ^. antennaReference_ecef_x
    , _msgBasePosEcef_y = fromEcefVal $ m ^. msg1006_reference ^. antennaReference_ecef_y
    , _msgBasePosEcef_z = fromEcefVal $ m ^. msg1006_reference ^. antennaReference_ecef_z
    }

fromMsg1012 :: MonadStore e m => Msg1012 -> m [MsgObs]
fromMsg1012 m = do
  let hdr = m ^. msg1012_header
  obs <- concatMapM fromObservation1012 $ m ^. msg1012_observations
  let chunks = zip [0..] $ chunksOf maxObsPerMessage obs
      totalMsgs = fromIntegral $ length chunks
  forM chunks . uncurry $ glonassChunkToMsgObs hdr totalMsgs


-- | Convert an RTCM 1019 GPS ephemeris message into an SBP MsgEphemerisGps.
fromMsg1019 :: MonadStore e m => Msg1019 -> m MsgEphemerisGps
fromMsg1019 m = do
  commonContent <- toGpsEphemerisCommonContent m
  toc           <- rtcmTimeToGpsTime (m ^. msg1019_ephemeris ^. gpsEphemeris_wn) (m ^. msg1019_ephemeris ^. gpsEphemeris_toc)
  return MsgEphemerisGps
    { _msgEphemerisGps_common   = commonContent
    , _msgEphemerisGps_tgd      =          applyScaleFactor 2 (-31) $ m ^. msg1019_ephemeris ^. gpsEphemeris_tgd
    , _msgEphemerisGps_c_rs     =          applyScaleFactor 2 (-5)  $ m ^. msg1019_ephemeris ^. gpsEphemeris_c_rs
    , _msgEphemerisGps_c_rc     =          applyScaleFactor 2 (-5)  $ m ^. msg1019_ephemeris ^. gpsEphemeris_c_rc
    , _msgEphemerisGps_c_uc     =          applyScaleFactor 2 (-29) $ m ^. msg1019_ephemeris ^. gpsEphemeris_c_uc
    , _msgEphemerisGps_c_us     =          applyScaleFactor 2 (-29) $ m ^. msg1019_ephemeris ^. gpsEphemeris_c_us
    , _msgEphemerisGps_c_ic     =          applyScaleFactor 2 (-29) $ m ^. msg1019_ephemeris ^. gpsEphemeris_c_ic
    , _msgEphemerisGps_c_is     =          applyScaleFactor 2 (-29) $ m ^. msg1019_ephemeris ^. gpsEphemeris_c_is
    , _msgEphemerisGps_dn       = gpsPi * (applyScaleFactor 2 (-43) $ m ^. msg1019_ephemeris ^. gpsEphemeris_dn)
    , _msgEphemerisGps_m0       = gpsPi * (applyScaleFactor 2 (-31) $ m ^. msg1019_ephemeris ^. gpsEphemeris_m0)
    , _msgEphemerisGps_ecc      =          applyScaleFactor 2 (-33) $ m ^. msg1019_ephemeris ^. gpsEphemeris_ecc
    , _msgEphemerisGps_sqrta    =          applyScaleFactor 2 (-19) $ m ^. msg1019_ephemeris ^. gpsEphemeris_sqrta
    , _msgEphemerisGps_omega0   = gpsPi * (applyScaleFactor 2 (-31) $ m ^. msg1019_ephemeris ^. gpsEphemeris_omega0)
    , _msgEphemerisGps_omegadot = gpsPi * (applyScaleFactor 2 (-43) $ m ^. msg1019_ephemeris ^. gpsEphemeris_omegadot)
    , _msgEphemerisGps_w        = gpsPi * (applyScaleFactor 2 (-31) $ m ^. msg1019_ephemeris ^. gpsEphemeris_w)
    , _msgEphemerisGps_inc      = gpsPi * (applyScaleFactor 2 (-31) $ m ^. msg1019_ephemeris ^. gpsEphemeris_i0)
    , _msgEphemerisGps_inc_dot  = gpsPi * (applyScaleFactor 2 (-43) $ m ^. msg1019_ephemeris ^. gpsEphemeris_idot)
    , _msgEphemerisGps_af0      =          applyScaleFactor 2 (-31) $ m ^. msg1019_ephemeris ^. gpsEphemeris_af0
    , _msgEphemerisGps_af1      =          applyScaleFactor 2 (-43) $ m ^. msg1019_ephemeris ^. gpsEphemeris_af1
    , _msgEphemerisGps_af2      =          applyScaleFactor 2 (-55) $ m ^. msg1019_ephemeris ^. gpsEphemeris_af2
    , _msgEphemerisGps_iodc     =                                     m ^. msg1019_ephemeris ^. gpsEphemeris_iodc
    , _msgEphemerisGps_iode     =                                     m ^. msg1019_ephemeris ^. gpsEphemeris_iode
    , _msgEphemerisGps_toc      = toc
    }

-- | Compute an SBP GLONASS ephemeris message from RTCM message 1020,
-- or 'Nothing' if the RTCM message is invalid or doesn't contain
-- enough information to form the SBP message.
ephemerisFromMsg1020 :: MonadStore e m => Msg1020 -> m (Maybe MsgEphemerisGlo)
ephemerisFromMsg1020 m = do
  commonContent <- glonassToEphemerisCommonContent m
  let eph = m ^. msg1020_ephemeris
      kmToMeters x = x * 1000
  return $ flip fmap commonContent $ \cc -> MsgEphemerisGlo
    { _msgEphemerisGlo_common = cc
    , _msgEphemerisGlo_gamma =
        applyScaleFactor 2 (-40) $ eph ^. glonassEphemeris_gammaN
    , _msgEphemerisGlo_tau =
        applyScaleFactor 2 (-30) $ eph ^. glonassEphemeris_tauN
    , _msgEphemerisGlo_pos =
        map (kmToMeters . applyScaleFactor 2 (-11))
        [ eph ^. glonassEphemeris_xn
        , eph ^. glonassEphemeris_yn
        , eph ^. glonassEphemeris_zn
        ]
    , _msgEphemerisGlo_vel =
        map (kmToMeters . applyScaleFactor 2 (-20))
        [ eph ^. glonassEphemeris_xndot
        , eph ^. glonassEphemeris_yndot
        , eph ^. glonassEphemeris_zndot
        ]
    , _msgEphemerisGlo_acc =
        map (kmToMeters . applyScaleFactor 2 (-30))
        [ eph ^. glonassEphemeris_xndotdot
        , eph ^. glonassEphemeris_yndotdot
        , eph ^. glonassEphemeris_zndotdot
        ]
    }

-- | Compute an SBP GLONASS frequency channel number mapping message
-- from RTCM message 1020, or 'Nothing' if the RTCM message is invalid
-- or doesn't contain enough information to form the SBP message.
--
-- This is a bit tricky.  The SBP message is expected to contain the
-- full table of satellite slot ID to FCN correspondences, but
-- supposedly setting any FCN to @0xFF@ means it will be ignored (this
-- happens by default for invalid slot IDs).  We only get one mapping
-- at a time from the RTCM stream, so we form messages with only a
-- single valid mapping.
fcnMapFromMsg1020 :: MonadStore e m => Msg1020 -> m (Maybe MsgFcnsGlo)
fcnMapFromMsg1020 m = do
  toe <- glonassEphemerisToGpsTime m
  let satNumField = m ^. msg1020_header . glonassEphemerisHeader_sat
      satFCNField = m ^. msg1020_header . glonassEphemerisHeader_channel
  return $ do
    satID <- fromIntegral <$> validateSatNum satNumField
    return MsgFcnsGlo
      { _msgFcnsGlo_wn     = toe ^. gpsTime_wn
      , _msgFcnsGlo_tow_ms = toe ^. gpsTime_tow
      , _msgFcnsGlo_fcns   = emptySlots & ix satID .~ satFCNField
      }
  where
    -- An FCN mapping with no information
    emptySlots = replicate 32 0xFF
    validateSatNum sn
      | sn < 1    = Nothing
      | sn > 29   = Nothing
      | otherwise = Just sn

-- | Compute both the GLONASS ephemeris and frequency channel mapping
-- messages from the given RTCM 1020 message.
handleMsg1020 :: MonadStore e m => Msg1020 -> m [SBPMsg]
handleMsg1020 m = do
  fcnmap <- maybeMessage SBPMsgFcnsGlo <$> fcnMapFromMsg1020 m
  ephem <- maybeMessage SBPMsgEphemerisGlo <$> ephemerisFromMsg1020 m
  return $ ephem <> fcnmap
  where
    sender = 0
    maybeMessage constr =
      maybe [] (\x -> [constr x $ toSBP x $ toSender sender])

-- | Convert an RTCM message into possibly multiple SBP messages.
--
convert :: MonadStore e m => RTCM3Msg -> m [SBPMsg]
convert = \case
  (RTCM3Msg1002 m _rtcm3) -> do
    let sender = m ^. msg1002_header ^. gpsObservationHeader_station
    m' <- fromMsg1002 m
    return $ flip fmap m' $ \x -> SBPMsgObs x $ toSBP x $ toSender sender
  (RTCM3Msg1004 m _rtcm3) -> do
    let sender = m ^. msg1004_header ^. gpsObservationHeader_station
    m' <- fromMsg1004 m
    return $ flip fmap m' $ \x -> SBPMsgObs x $ toSBP x $ toSender sender
  (RTCM3Msg1005 m _rtcm3) -> do
    let sender =  m ^. msg1005_reference ^. antennaReference_station
    m' <- fromMsg1005 m
    return [SBPMsgBasePosEcef m' $ toSBP m' $ toSender sender]
  (RTCM3Msg1006 m _rtcm3) -> do
    let sender = m ^. msg1006_reference ^. antennaReference_station
    m' <- fromMsg1006 m
    return [SBPMsgBasePosEcef m' $ toSBP m' $ toSender sender]
  (RTCM3Msg1012 m _rtcm3) -> do
    let sender = m ^. msg1012_header ^. glonassObservationHeader_station
    m' <- fromMsg1012 m
    return $ flip fmap m' $ \x -> SBPMsgObs x $ toSBP x $ toSender sender
  (RTCM3Msg1013 m _rtcm3) -> do
    wn <- view storeWn
    liftIO $ writeIORef wn $ toWn $ m ^. msg1013_header ^. messageHeader_mjd
    return mempty
  (RTCM3Msg1019 m _rtcm3) -> do
    let sender = 0
    m' <- fromMsg1019 m
    return [SBPMsgEphemerisGps m' $ toSBP m' $ toSender sender]
  (RTCM3Msg1020 m _rtcm3) ->
    -- This can produce more than one output message per input
    -- message, so we have a helper function.
    handleMsg1020 m
  _rtcm3Msg -> return mempty

newStore :: IO Store
newStore = do
  t <- getCurrentTime
  let gpsDays = diffDays (utctDay t) (fromGregorian 1980 1 6)
      wn = fromIntegral $ div gpsDays 7
  Store <$> newIORef wn <*> newIORef mempty <*> newIORef t
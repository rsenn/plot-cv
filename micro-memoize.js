function _define_property(obj, key, value) {
    if (key in obj) {
        Object.defineProperty(obj, key, {
            value: value,
            enumerable: true,
            configurable: true,
            writable: true
        });
    } else {
        obj[key] = value;
    }
    return obj;
}
// utils
import { cloneArray } from './utils';
export class Cache {
    /**
   * The number of cached [key,value] results.
   */ get size() {
        return this.keys.length;
    }
    /**
   * A copy of the cache at a moment in time. This is useful
   * to compare changes over time, since the cache mutates
   * internally for performance reasons.
   */ get snapshot() {
        return {
            keys: cloneArray(this.keys),
            size: this.size,
            values: cloneArray(this.values)
        };
    }
    /**
   * Gets the matching key index when a custom key matcher is used.
   */ _getKeyIndexFromMatchingKey(keyToMatch) {
        const { isMatchingKey, maxSize } = this.options;
        const { keys } = this;
        const keysLength = keys.length;
        if (!keysLength) {
            return -1;
        }
        if (isMatchingKey(keys[0], keyToMatch)) {
            return 0;
        }
        if (maxSize > 1) {
            for(let index = 1; index < keysLength; index++){
                if (isMatchingKey(keys[index], keyToMatch)) {
                    return index;
                }
            }
        }
        return -1;
    }
    /**
   * Gets the matching key index when multiple keys are used.
   */ _getKeyIndexForMany(keyToMatch) {
        const { isEqual } = this.options;
        const { keys } = this;
        const keysLength = keys.length;
        if (!keysLength) {
            return -1;
        }
        if (keysLength === 1) {
            return this._getKeyIndexForSingle(keyToMatch);
        }
        const keyLength = keyToMatch.length;
        let existingKey;
        let argIndex;
        if (keyLength > 1) {
            for(let index = 0; index < keysLength; index++){
                existingKey = keys[index];
                if (existingKey.length === keyLength) {
                    argIndex = 0;
                    for(; argIndex < keyLength; argIndex++){
                        if (!isEqual(existingKey[argIndex], keyToMatch[argIndex])) {
                            break;
                        }
                    }
                    if (argIndex === keyLength) {
                        return index;
                    }
                }
            }
        } else {
            for(let index = 0; index < keysLength; index++){
                existingKey = keys[index];
                if (existingKey.length === keyLength && isEqual(existingKey[0], keyToMatch[0])) {
                    return index;
                }
            }
        }
        return -1;
    }
    /**
   * Gets the matching key index when a single key is used.
   */ _getKeyIndexForSingle(keyToMatch) {
        const { keys } = this;
        if (!keys.length) {
            return -1;
        }
        const existingKey = keys[0];
        const { length } = existingKey;
        if (keyToMatch.length !== length) {
            return -1;
        }
        const { isEqual } = this.options;
        if (length > 1) {
            for(let index = 0; index < length; index++){
                if (!isEqual(existingKey[index], keyToMatch[index])) {
                    return -1;
                }
            }
            return 0;
        }
        return isEqual(existingKey[0], keyToMatch[0]) ? 0 : -1;
    }
    /**
   * Order the array based on a Least-Recently-Used basis.
   */ orderByLru(key, value, startingIndex) {
        const { keys } = this;
        const { values } = this;
        const currentLength = keys.length;
        let index = startingIndex;
        while(index--){
            keys[index + 1] = keys[index];
            values[index + 1] = values[index];
        }
        keys[0] = key;
        values[0] = value;
        const { maxSize } = this.options;
        if (currentLength === maxSize && startingIndex === currentLength) {
            keys.pop();
            values.pop();
        } else if (startingIndex >= maxSize) {
            // eslint-disable-next-line no-multi-assign
            keys.length = values.length = maxSize;
        }
    }
    /**
   * Update the promise method to auto-remove from cache if rejected, and
   * if resolved then fire cache hit / changed.
   */ updateAsyncCache(memoized) {
        const { onCacheChange, onCacheHit } = this.options;
        const [firstKey] = this.keys;
        const [firstValue] = this.values;
        this.values[0] = firstValue.then((value)=>{
            if (this.shouldUpdateOnHit) {
                onCacheHit(this, this.options, memoized);
            }
            if (this.shouldUpdateOnChange) {
                onCacheChange(this, this.options, memoized);
            }
            return value;
        }, (error)=>{
            const keyIndex = this.getKeyIndex(firstKey);
            if (keyIndex !== -1) {
                this.keys.splice(keyIndex, 1);
                this.values.splice(keyIndex, 1);
            }
            throw error;
        });
    }
    constructor(options){
        _define_property(this, "canTransformKey", void 0);
        _define_property(this, "getKeyIndex", void 0);
        _define_property(this, "options", void 0);
        _define_property(this, "shouldCloneArguments", void 0);
        _define_property(this, "shouldUpdateOnAdd", void 0);
        _define_property(this, "shouldUpdateOnChange", void 0);
        _define_property(this, "shouldUpdateOnHit", void 0);
        /**
   * The prevents call arguments which have cached results.
   */ _define_property(this, "keys", void 0);
        /**
   * The results of previous cached calls.
   */ _define_property(this, "values", void 0);
        this.keys = [];
        this.values = [];
        this.options = options;
        const isMatchingKeyFunction = typeof options.isMatchingKey === 'function';
        if (isMatchingKeyFunction) {
            this.getKeyIndex = this._getKeyIndexFromMatchingKey;
        } else if (options.maxSize > 1) {
            this.getKeyIndex = this._getKeyIndexForMany;
        } else {
            this.getKeyIndex = this._getKeyIndexForSingle;
        }
        this.canTransformKey = typeof options.transformKey === 'function';
        this.shouldCloneArguments = this.canTransformKey || isMatchingKeyFunction;
        this.shouldUpdateOnAdd = typeof options.onCacheAdd === 'function';
        this.shouldUpdateOnChange = typeof options.onCacheChange === 'function';
        this.shouldUpdateOnHit = typeof options.onCacheHit === 'function';
    }
}

import { Cache } from './Cache';
import { cloneArray, getCustomOptions, isMemoized, isSameValueZero, mergeOptions } from './utils';
function createMemoizedFunction(fn, options = {}) {
    if (isMemoized(fn)) {
        return createMemoizedFunction(fn.fn, mergeOptions(fn.options, options));
    }
    if (typeof fn !== 'function') {
        throw new TypeError('You must pass a function to `memoize`.');
    }
    const { isEqual = isSameValueZero, isMatchingKey, isPromise = false, maxSize = 1, onCacheAdd, onCacheChange, onCacheHit, transformKey } = options;
    const normalizedOptions = mergeOptions({
        isEqual,
        isMatchingKey,
        isPromise,
        maxSize,
        onCacheAdd,
        onCacheChange,
        onCacheHit,
        transformKey
    }, getCustomOptions(options));
    const cache = new Cache(normalizedOptions);
    const { keys, values, canTransformKey, shouldCloneArguments, shouldUpdateOnAdd, shouldUpdateOnChange, shouldUpdateOnHit } = cache;
    const memoized = function() {
        let key = shouldCloneArguments ? cloneArray(arguments) : arguments;
        if (canTransformKey) {
            key = transformKey(key);
        }
        const keyIndex = keys.length ? cache.getKeyIndex(key) : -1;
        if (keyIndex !== -1) {
            if (shouldUpdateOnHit) {
                onCacheHit(cache, normalizedOptions, memoized);
            }
            if (keyIndex) {
                cache.orderByLru(keys[keyIndex], values[keyIndex], keyIndex);
                if (shouldUpdateOnChange) {
                    onCacheChange(cache, normalizedOptions, memoized);
                }
            }
        } else {
            const newValue = fn.apply(this, arguments);
            const newKey = shouldCloneArguments ? key : cloneArray(arguments);
            cache.orderByLru(newKey, newValue, keys.length);
            if (isPromise) {
                cache.updateAsyncCache(memoized);
            }
            if (shouldUpdateOnAdd) {
                onCacheAdd(cache, normalizedOptions, memoized);
            }
            if (shouldUpdateOnChange) {
                onCacheChange(cache, normalizedOptions, memoized);
            }
        }
        return values[0];
    };
    memoized.cache = cache;
    memoized.fn = fn;
    memoized.isMemoized = true;
    memoized.options = normalizedOptions;
    return memoized;
}
export default createMemoizedFunction;

/**
 * @constant DEFAULT_OPTIONS_KEYS the default options keys
 */ const DEFAULT_OPTIONS_KEYS = {
    isEqual: true,
    isMatchingKey: true,
    isPromise: true,
    maxSize: true,
    onCacheAdd: true,
    onCacheChange: true,
    onCacheHit: true,
    transformKey: true
};
/**
 * @function slice
 *
 * @description
 * slice.call() pre-bound
 */ export const { slice } = Array.prototype;
/**
 * @function cloneArray
 *
 * @description
 * clone the array-like object and return the new array
 *
 * @param arrayLike the array-like object to clone
 * @returns the clone as an array
 */ export function cloneArray(arrayLike) {
    const { length } = arrayLike;
    if (!length) {
        return [];
    }
    if (length === 1) {
        return [
            arrayLike[0]
        ];
    }
    if (length === 2) {
        return [
            arrayLike[0],
            arrayLike[1]
        ];
    }
    if (length === 3) {
        return [
            arrayLike[0],
            arrayLike[1],
            arrayLike[2]
        ];
    }
    return slice.call(arrayLike, 0);
}
/**
 * @function getCustomOptions
 *
 * @description
 * get the custom options on the object passed
 *
 * @param options the memoization options passed
 * @returns the custom options passed
 */ export function getCustomOptions(options) {
    const customOptions = {};
    /* eslint-disable no-restricted-syntax */ for(const key in options){
        if (!DEFAULT_OPTIONS_KEYS[key]) {
            customOptions[key] = options[key];
        }
    }
    /* eslint-enable */ return customOptions;
}
/**
 * @function isMemoized
 *
 * @description
 * is the function passed already memoized
 *
 * @param fn the function to test
 * @returns is the function already memoized
 */ export function isMemoized(fn) {
    return typeof fn === 'function' && fn.isMemoized;
}
/**
 * @function isSameValueZero
 *
 * @description
 * are the objects equal based on SameValueZero equality
 *
 * @param object1 the first object to compare
 * @param object2 the second object to compare
 * @returns are the two objects equal
 */ export function isSameValueZero(object1, object2) {
    // eslint-disable-next-line no-self-compare
    return object1 === object2 || object1 !== object1 && object2 !== object2;
}
/**
 * @function mergeOptions
 *
 * @description
 * merge the options into the target
 *
 * @param existingOptions the options provided
 * @param newOptions the options to include
 * @returns the merged options
 */ export function mergeOptions(existingOptions, newOptions) {
    const target = {};
    /* eslint-disable no-restricted-syntax */ for(const key in existingOptions){
        target[key] = existingOptions[key];
    }
    for(const key in newOptions){
        target[key] = newOptions[key];
    }
    /* eslint-enable */ return target;
}